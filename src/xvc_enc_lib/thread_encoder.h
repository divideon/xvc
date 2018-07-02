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

#ifndef XVC_ENC_LIB_THREAD_ENCODER_H_
#define XVC_ENC_LIB_THREAD_ENCODER_H_

// Some C++11 headers are not allowed by cpplint
#include <condition_variable>   // NOLINT
#include <deque>
#include <functional>
#include <list>
#include <memory>
#include <mutex>                // NOLINT
#include <thread>               // NOLINT
#include <vector>

#include "xvc_common_lib/segment_header.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/picture_encoder.h"

namespace xvc {

class ThreadEncoder {
public:
  using PicEncList = std::vector<std::shared_ptr<const PictureEncoder>>;
  using PictureDecodedCallback =
    std::function<void(std::shared_ptr<PictureEncoder>, const PicEncList &,
                       std::unique_ptr<std::vector<uint8_t>> pic_nal)>;

  ThreadEncoder(int num_threads, const EncoderSettings &encoder_settings);
  ~ThreadEncoder();
  size_t GetNumThreads() const { return worker_threads_.size(); }
  void StopAll();
  void EncodeAsync(std::shared_ptr<SegmentHeader> segment_header,
                   std::shared_ptr<PictureEncoder> pic_enc,
                   const std::vector<std::shared_ptr<const PictureEncoder>> &,
                   std::unique_ptr<std::vector<uint8_t>> &&output_nal_buffer,
                   int segment_qp, bool buffer_flag);
  void WaitForPicture(const std::shared_ptr<PictureEncoder> &pic,
                      PictureDecodedCallback callback);
  void WaitOne(PictureDecodedCallback callback);

private:
  struct WorkItem {
    std::shared_ptr<PictureEncoder> pic_enc;
    std::vector<std::shared_ptr<const PictureEncoder>> pic_dependencies;
    std::shared_ptr<SegmentHeader> segment_header;
    std::unique_ptr<std::vector<uint8_t>> nal_buffer;
    int segment_qp;
    bool buffer_flag;
    const std::vector<uint8_t> *pic_bytes;
  };
  void WorkerMain(int thread_idx);

  const EncoderSettings &encoder_settings_;
  std::vector<std::thread> worker_threads_;
  std::mutex global_mutex_;
  std::condition_variable wait_work_cond_;
  std::condition_variable work_done_cond_;
  std::list<WorkItem> pending_work_;
  std::deque<WorkItem> finished_work_;
  bool running_ = true;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_THREAD_ENCODER_H_
