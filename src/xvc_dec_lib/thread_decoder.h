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

#ifndef XVC_DEC_LIB_THREAD_DECODER_H_
#define XVC_DEC_LIB_THREAD_DECODER_H_

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
#include "xvc_dec_lib/picture_decoder.h"

namespace xvc {

class ThreadDecoder {
public:
  using PicDecList = std::vector<std::shared_ptr<const PictureDecoder>>;
  using PictureDecodedCallback =
    std::function<void(std::shared_ptr<PictureDecoder>, bool,
                       const PicDecList &)>;

  explicit ThreadDecoder(int num_threads);
  ~ThreadDecoder();
  void StopAll();
  void DecodeAsync(std::shared_ptr<SegmentHeader> &&segment_header,
                   std::shared_ptr<SegmentHeader> &&prev_segment_header,
                   std::shared_ptr<PictureDecoder> &&pic_dec,
                   std::vector<std::shared_ptr<const PictureDecoder>> &&deps,
                   std::unique_ptr<std::vector<uint8_t>> &&nal,
                   size_t nal_offset);
  void WaitForPicture(const std::shared_ptr<PictureDecoder> &pic,
                      PictureDecodedCallback callback);
  void WaitOne(PictureDecodedCallback callback);
  void WaitAll(PictureDecodedCallback callback);

private:
  struct WorkItem {
    std::shared_ptr<PictureDecoder> pic_dec;
    std::vector<std::shared_ptr<const PictureDecoder>> inter_dependencies;
    std::shared_ptr<SegmentHeader> segment_header;
    std::shared_ptr<SegmentHeader> prev_segment_header;
    std::unique_ptr<std::vector<uint8_t>> nal;
    std::size_t nal_offset;
    bool success;
  };
  void WorkerMain();

  std::vector<std::thread> worker_threads_;
  std::mutex global_mutex_;
  std::condition_variable wait_work_cond_;
  std::condition_variable work_done_cond_;
  std::list<WorkItem> pending_work_;
  std::deque<WorkItem> finished_work_;
  int jobs_in_flight_ = 0;
  bool running_ = true;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_THREAD_DECODER_H_
