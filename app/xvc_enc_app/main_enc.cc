/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_app/encoder_app.h"

int main(int argc, const char* argv[]) {
  xvc_app::EncoderApp main_app;

  main_app.ReadArguments(argc, argv);
  main_app.CheckParameters();
  main_app.CreateAndConfigureApi();
  main_app.PrintEncoderSettings();
  main_app.MainEncoderLoop();
  main_app.CloseStream();
  main_app.PrintStatistics();

  return 0;
}
