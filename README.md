# xvc codec [![AppVeyor Build Status](https://ci.appveyor.com/api/projects/status/github/divideon/xvc?branch=master&svg=true)](https://ci.appveyor.com/project/divideon/xvc)

The xvc codec is a next-generation software-defined video compression format, built using
world-class compression technologies.

The xvc software is available under a dual-licensing scheme with an LGPL option and a
commercial option.

Visit [xvc.io](https://xvc.io) for more information.

The master branch represents the current official version of xvc. Development
of new versions of xvc happens in the dev branch.

## Building instructions

### Prerequisites

 1. [CMake](https://cmake.org) version 3.1 or higher.
 2. [Git](https://git-scm.com/).
 3. C++ compiler with C++11 support

The following C++11 compilers have been known to work:

 * Visual Studio 2015 or later
 * GCC 4.8 or later
 * Clang 3.3 or later

### Linux build steps

The following commands will checkout the project source code and create a
directory called 'build' where the compiler output will be placed.
CMake is then used for generating build files and compiling the xvc binaries.

    $ git clone https://github.com/divideon/xvc.git
    $ cd xvc
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

This will create xvc encoder and decoder binaries in the xvc/build/app/ folder.

### Windows build steps

The following commands will checkout the project source code and create a
directory called 'build' where the compiler output will be placed.
CMake is then used for generating build files and creating the Visual Studio
solution.

    $ git clone https://github.com/divideon/xvc.git
    $ cd xvc
    $ mkdir build
    $ cd build
    $ cmake -G "Visual Studio 14 2015 Win64" ..

The -G argument should be adjusted to match your version of Visual Studio and
the target architecture.
This will generate a Visual Studio solution called xvc.sln in the build folder.
After building the solution, the xvc encoder and decoder binaries will be found
in the xvc/build/app/Release/ folder.

## Testing the xvc codec

### Command line encoder and decoder

For generating xvc bitstreams the following command shows typical usage:

    $ xvcenc -input-file input.yuv -input-width 1920 -input-height 1080 \
        -framerate 30 -output-file mybitstream.xvc -qp 32

For decoding an xvc bitstream use the xvcdec application:

    $ xvcdec -bitstream-file mybitstream.xvc -output-file decoded.yuv

Both xvcenc and xvcdec supports reading and writing y4m output.

Pipes can be used together with e.g. ffmpeg to stream y4m input to encoder:

    $ ffmpeg -i movie.mkv -f yuv4mpegpipe - | xvcenc -input-file - \
        -qp 30 -output-file mybitstream.xvc

On the decoder, pipes can also be used to stream decoded output fo ffplay:

    $ xvcdec -bitstream-file mybitstream.xvc -output-file - | ffplay -i -

### Command line syntax

To show all available encoder arguments run:

	$ xvcenc -help

For decoder arguments run:

	$ xvcdec -help

## Coding style

The xvc source code follows the [Google C++ Style Guide](
https://google.github.io/styleguide/cppguide.html).

## Support

For questions/comments please email support@xvc.io

## Contributing

Patches are encouraged, and may be submitted by forking this project and
submitting a pull request through GitHub.

Please visit [xvc.io](https://xvc.io/developers) for more information on how
to become an xvc contributor.

