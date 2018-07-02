/******************************************************************************
* Copyright (C) 2017, Divideon.
*
* Redistribution and use in source and binary form, with or without
* modifications is permitted only under the terms and conditions set forward
* in the xvc License Agreement. For commercial redistribution and use, you are
* required to send a signed copy of the xvc License Agreement to Divideon.
*
* Redistribution and use in source and binary form is permitted free of charge
* for non-commercial purposes. See definition of non-commercial in the xvc
* License Agreement.
*
* All redistribution of source code must retain this copyright notice
* unmodified.
*
* The xvc License Agreement is available at https://xvc.io/license/.
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
