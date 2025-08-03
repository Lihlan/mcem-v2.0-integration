#pragma once
/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2019-2033, Audio Video coding Standard Workgroup of China
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of Audio Video coding Standard Workgroup of China
*    nor the names of its contributors maybe used to endorse or promote products
*    derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "commonDef.h"
#include <fstream>
#include<vector>

using namespace std;
//#define MAX_BS_BUF 1 << 28        ///< 256 * 1024 * 1024 bytes
#define MAX_HEADER_BS_BUF 1 << 16  ///< 256 * 256 bytes


/**
 * Class TComBitstream
 * bitstream buffer (a pointer to the real buffer)
 */

#define start_code_lower                  (0x00)
#define start_code_upper                  (0x04)


#define sequence_start_code               (0x00)
#define sequence_end_code                 (0x01)
//#define geometry_start_code               (0x02)
//#define attribute_start_code              (0x03)
//#define frame_start_code                  (0x04)
//#define user_data_start_code              (0x05)
//#define geometry_slice_header_start_code  (0x06)
//#define color_slice_header_start_code     (0x07)
//#define refl_slice_header_start_code      (0x08)
#define mesh_payload_start_code           (0x02)
#define geometry_payload_start_code       (0x03)
#define texture_payload_start_code        (0x04)
//#define refl_slice_payload_start_code     (0x0B)

class TComBitstream {
public:
  /* user space address indicating buffer */
  void* addr;
  void* addr2;
  /* physical address indicating buffer, if any */
  void* pddr;
  /* byte size of buffer memory */
  int bsize;
  /* byte size of bitstream in buffer */
  int ssize;
  /* bitstream has an error? */
  int err;
  /* arbitrary data, if needs */
  int ndata[4];
  /* arbitrary address, if needs */
  void* pdata[4];
  /* time-stamps */
  long long ts[4];

public:
  TComBitstream();
  ~TComBitstream();

  void reset();
  void allocateBuffSize(const size_t buffSize);
  void* getBitStreamBuffer();

private:
  void init();
};  ///< END CLASS TComBitstream

class TComBufferChunk : public TComBitstream {
private:
    BufferChunkType m_bufferType;

public:
	TComBufferChunk() = default;
    TComBufferChunk(BufferChunkType bufferType);
	~TComBufferChunk() = default;

	void writeToBitstream(ofstream* outBitstream, uint64_t length);
    void writeStartCodeToBitstream(ofstream* outBitstream);
    void writeFinalCodeToBitstream(ofstream* outBitstream);
    BufferChunkType determineBufferType(const uint32_t& startCode);
	int readFromBitstream(ifstream& inBitstream, int buffersize, bool& decodeSequence, uint8_t& nextStartCode);
    void setBufferType(BufferChunkType bufferType);
    BufferChunkType getBufferType();

private:
    int readBufferChunk(ifstream& inBitstream, size_t& bufferChunkSize, uint8_t& nextStartCode);
	size_t initParsingConvertPayloadToRBSP(const size_t uiBytesRead, unsigned char* pBuffer, unsigned char* pBuffer2);
};
/**
 * Class COM_BS
 * bitstream buffer (real buffer)
 */

class COM_BS;
typedef int (*COM_BS_FN_FLUSH)(COM_BS* bs);

class COM_BS {
public:
  /* buffer */
  uint32_t code;
  /* bits left in buffer */
  int leftbits;
  /*! address of current writing position */
  uint8_t* cur;
  /*! address of bitstream buffer end */
  uint8_t* end;
  /*! address of bitstream buffer begin */
  uint8_t* beg;
  /*! address of temporal buffer for demulation begin */
  uint8_t* buftmp;
  /*! size of bitstream buffer in byte */
  int size;
  /*! address of function for flush */
  COM_BS_FN_FLUSH fn_flush;
  /*! arbitrary data, if needs */
  int ndata[4];
  /*! arbitrary address, if needs */
  void* pdata[4];
  // avs2
  uint8_t* p_start; /* actual buffer for written bytes */
  uint8_t* p;       /* pointer to byte written currently */
  uint8_t* p_end;   /* end of the actual buffer */
  int i_left;     /* current bit counter to go */

public:
  int com_bsw_get_write_byte() {
    return (int)(cur - beg);  ///< get number of byte written
  }
  int com_bsw_get_sink_byte()  ///< number of bytes to be sunk
  {
    return ((32 - leftbits + 7) >> 3);
  }
  bool com_is_byte_align()  ///< check if byte is aligned
  {
    return !(leftbits & 0x7);
  }
  void init() {
    code = 0;
    leftbits = 0;
    cur = end = beg = buftmp = nullptr;
    size = 0;
    fn_flush = 0;
    memset(ndata, 0, 4 * sizeof(int));
    memset(pdata, 0, 4 * sizeof(void*));
  }
};  ///< END CLASS COM_BS

///< \}
