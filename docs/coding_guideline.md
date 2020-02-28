
## 1. Language

Use C++ and in general follow the guidelines at: https://google.github.io/styleguide/cppguide.html

The cpplint tool can be used to check formatting: https://pypi.python.org/pypi/cpplint

Example:

cpplint --quiet --recursive --filter=-whitespace/indent --repository="app" app

cpplint --quiet --recursive --filter=-whitespace/indent --repository="src" src

cpplint --quiet --recursive --filter=-whitespace/indent test/xvc_test/*_test.cc

Cppcheck can be used for statistical analysis: http://cppcheck.sourceforge.net/

Example:

cppcheck --quiet --enable=all --std=c++11 -DHM_STRICT --suppress=knownConditionTrueFalse --suppress=unusedFunction --suppress=missingInclude --suppress=useStlAlgorithm -Isrc src app

### 1.1. Header multiple include safety

All header files include a mechanism to prevent multiple inclusion by using a #define.  

For a header file named motion_estimation.h the structure would be the following:
```c++
#ifndef XVC_ENC_LIB_MOTION_ESTIMATION_H_
#define XVC_ENC_LIB_MOTION_ESTIMATION_H_
<content of header file>
#endif
```
### 1.2. Initialization

Use Initialization inline with member declaration.

I.e.: 
```c++
class Foo
{
        Foo() {}
        Int bar_ = 0;
};
```
is preferred over:
```c++
class Foo
{
        Foo() { bar_ = 0; }
        Int bar_;
};
```
and
```c++
class Foo
{
        Foo() bar_(0) {}
        Int bar_;
};
```
### 1.3. Namespaces

Use the following in both .h and .cc files of the lib:
```c++
namespace xvc {

} // namespace xvc
```
Use the following in both .h and .cc files of the app:
```c++
namespace xvc_app {

} // namespace xvc_app
```
### 1.4. Indentation of private and public

Do not use indentation for the keywords "private" and "public".

### 1.5. Position of asterix

For pointers, the asterix shall be attached to the variable not the type i.e. to the right:
void *somePointer instead of void* somePointer

The only exception is on return types of functions where the asterix shall be attached to the return type and not the function i.e. to the left.

## 2. Naming

### 2.1. Terminology

Use the words/phrases on the left, not the ones on the right.

* coding unit - treeblock, macroblock, LCU, tree block, coding block, codingunit, prediction unit, prediction block, predictionunit, transform unit, transform block, transformunit
* picture - frame
* framerate - frame rate
* bitrate - bit rate
* bitstream - bit stream
* bytestream - byte stream
* bitdepth - bit depth
* bit reader - bitreader
* bit writer - bitwriter
* chroma format - chromaformat

Use the following abbreviations when it is not in a class-name/file-name/typedef and when it is clear what it represents (otherwise use the whole word):
* orig - original
* resi - residual
* pred - prediction
* bipred - biprediction
* reco - reconstruction (sample)
* enc - encoded
* dec - decoded
* rec - reconstructed (picture)
* pic - picture (picture is used in the App but not in the Lib)

deblock - is the primary term used for the deblocking filter except for the file and the class which are called "deblocking".

bytes - is used for talking about memory arrays (typically containing sample values - bitdepth and chroma format must be known in order to interpret the bytes as samples)

data - is used for additional associated data such as resolution of an image 

sample - is used for the individual elements of a picture instead of pixels

The term Gop is used for all pictures of one segment and always starts (in decoding order) with an Intra picture.

The term Sub Gop is used for all pictures of a sub-segment (in decoding order).

The term "picture order count" is used to represent the pictures in output order.

The term "decoding order count" is used to represent the pictures in decoding order.

Intra Access Picture (IAP) - An intra picture that provides tune-in / random access functionality.

Use Y, U and V for the differnt components. Never use the terms Cb or Cr.

The term "compress" is used to represent the action of creating the encoded representation of something.

The term "write" is used to represent the action of putting an encoded representation to a bitstream.

The term "encode" is used to represent the action of compressing and writing.

On the decoder side the corresponding terms, "decompress", "read" and "decode" are used.

American english is used, e.g. signaling and color (instead of signalling and colour)

### 2.2. Abbreviations

Preferably use full names in classes and variable names but for obvious abbreviations it is ok, e.g. RefPic instead of ReferencePicture.

Use CamelCase also for abbreviations such as Dct and Qp.

### 2.3. File names

Use .cc and use lowercase letters with underscore to separate words e.g.

motion_estimation.cc

The follwoing files will be generated as output when building the xvc solution:
```
xvcenc.exe
xvcdec.exe
xvcenc.lib
xvcdec.lib
```
The header files containing the API are called:
```
xvcenc.h
xvcdec.h
```
### 2.4. Project names

The project files are called:
```
xvc_enc_app
xvc_dec_app
xvc_enc_lib
xvc_dec_lib
xvc_common_lib
```
