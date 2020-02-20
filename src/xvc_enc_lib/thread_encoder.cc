/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
******************************************************************************/

#include "xvc_enc_lib/thread_encoder.h"

#include <algorithm>
#include <utility>

namespace xvc {

static const int kMaxNumThreads = 64;

ThreadEncoder::ThreadEncoder(int num_threads,
                             const EncoderSettings &encoder_settings)
  : encoder_settings_(encoder_settings) {
  if (num_threads < 0) {
    num_threads = std::thread::hardware_concurrency();
  }
  // Need at least one thread to work
  num_threads = std::min(std::max(1, num_threads), kMaxNumThreads);
  for (int thread_idx = 0; thread_idx < num_threads; thread_idx++) {
    worker_threads_.emplace_back([this, thread_idx] {
      WorkerMain(thread_idx);
    });
  }
}

ThreadEncoder::~ThreadEncoder() {
  StopAll();
}

void ThreadEncoder::StopAll() {
  std::unique_lock<std::mutex> lock(global_mutex_);
  running_ = false;
  wait_work_cond_.notify_all();  // wakeup all
  lock.unlock();
  for (auto &thread : worker_threads_) {
    thread.join();
  }
  worker_threads_.clear();
}

void ThreadEncoder::EncodeAsync(
  std::shared_ptr<SegmentHeader> segment_header,
  std::shared_ptr<PictureEncoder> pic_enc,
  const std::vector<std::shared_ptr<const PictureEncoder>> &deps,
  std::unique_ptr<std::vector<uint8_t>> &&output_nal_buffer,
  int segment_qp, bool buffer_flag) {
  // Prepare work for thread
  WorkItem work;
  work.pic_enc = std::move(pic_enc);
  work.pic_dependencies = std::move(deps);
  work.segment_header = std::move(segment_header);
  work.nal_buffer = std::move(output_nal_buffer);
  work.segment_qp = segment_qp;
  work.buffer_flag = buffer_flag;

  // Signal one worker thread to begin processing
  std::unique_lock<std::mutex> lock(global_mutex_);
  pending_work_.push_back(std::move(work));
  wait_work_cond_.notify_one();
}

void ThreadEncoder::WaitForPicture(const std::shared_ptr<PictureEncoder> &pic,
                                   PictureDecodedCallback callback) {
  while (pic->GetOutputStatus() != OutputStatus::kHasNotBeenOutput) {
    WaitOne(callback);
  }
}

void ThreadEncoder::WaitOne(PictureDecodedCallback callback) {
  std::unique_lock<std::mutex> lock(global_mutex_);
  work_done_cond_.wait(lock, [this] { return !finished_work_.empty(); });
  WorkItem work = std::move(finished_work_.front());
  finished_work_.pop_front();
  // Note! Callback invoked while lock is being held
  callback(work.pic_enc, work.pic_dependencies, std::move(work.nal_buffer));
}

void ThreadEncoder::WorkerMain(int thread_idx) {
  bool restrictions_loaded = false;

  std::unique_lock<std::mutex> lock(global_mutex_);
  while (true) {
    ThreadEncoder::WorkItem work;
    // Find one picture that can be decoded now
    while (running_) {
      // Verify all dependencies are satisfied before taking work
      auto best_it = pending_work_.end();
      for (auto it = pending_work_.begin(); it != pending_work_.end(); ++it) {
        bool valid = true;
        for (auto &dependency : it->pic_dependencies) {
          if (dependency->GetOutputStatus() == OutputStatus::kProcessing ||
              dependency->GetOutputStatus() == OutputStatus::kReady) {
            valid = false;
            break;
          }
        }
        if (!valid) {
          continue;
        }
        if (best_it == pending_work_.end() ||
            it->pic_enc->GetTid() < best_it->pic_enc->GetTid()) {
          best_it = it;
        }
      }
      if (best_it != pending_work_.end()) {
        work = std::move(*best_it);
        pending_work_.erase(best_it);
        break;
      }
      wait_work_cond_.wait(lock);
    }
    if (!running_) {
      break;
    }
    lock.unlock();

    if (!restrictions_loaded) {
      // The assumption is that restrictions never change during encoding
      restrictions_loaded = true;
      Restrictions::GetRW() = work.segment_header->restrictions;
    }

    // Encode picture
    const std::vector<uint8_t> *pic_bytes =
      work.pic_enc->Encode(*work.segment_header, work.segment_qp,
                           work.buffer_flag, encoder_settings_);
    *work.nal_buffer = *pic_bytes;
    work.pic_enc->SetOutputStatus(OutputStatus::kFinishedProcessing);

    lock.lock();
    // Notify all workers that a dependency might be ready
    wait_work_cond_.notify_all();
    // Notify main thread picture that picture is fully decoded
    // TODO(PH) some fields are not needed anymore (like nal)
    finished_work_.push_back(std::move(work));
    work_done_cond_.notify_all();
  }
}

}   // namespace xvc
