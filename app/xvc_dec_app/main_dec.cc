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

#include "xvc_dec_app/decoder_app.h"

int main(int argc, const char *argv[]) {
  xvc_app::DecoderApp main_app;

  main_app.ReadArguments(argc, argv);
  main_app.CheckParameters();
  main_app.CreateAndConfigureApi();
  main_app.PrintDecoderSettings();
  main_app.MainDecoderLoop();
  main_app.CloseStream();
  main_app.PrintStatistics();

  return main_app.CheckConformance();
}
