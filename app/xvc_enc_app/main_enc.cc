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

#include "xvc_enc_app/encoder_app.h"

int main(int argc, const char* argv[]) {
  xvc_app::EncoderApp main_app;

  main_app.ReadArguments(argc, argv);
  main_app.CheckParameters();
  main_app.PrintEncoderSettings();
  main_app.MainEncoderLoop();
  main_app.PrintStatistics();

  return 0;
}
