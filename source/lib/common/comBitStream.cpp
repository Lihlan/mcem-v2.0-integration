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

#include "comBitStream.h"

///< \in TLibCommon

/**
 * Implementation of TComBitstream
 * bitstream buffer operations
 */

//////////////////////////////////////////////////////////////////////////
// Public class functions
//////////////////////////////////////////////////////////////////////////

TComBitstream::TComBitstream() {
  init();
}

TComBitstream::~TComBitstream() {
  reset();
}

void TComBitstream::reset() {
  if (addr)
    free((unsigned char*)addr);
  if (addr2)
    free((unsigned char*)addr2);
  init();
}

void TComBitstream::allocateBuffSize(const size_t buffSize) {
  ///< allocate bitstream buffer
  if (addr == nullptr) {
    addr = (unsigned char*)calloc(1, buffSize);
    assert(checkCond(addr != NULL, "Error: cannot allocate bitstream buffer!"));
  }
  if (addr2 == nullptr) {
    addr2 = (unsigned char*)calloc(1, buffSize);
    assert(checkCond(addr2 != NULL, "Error: cannot allocate bitstream buffer!"));
  }
  ssize = (int)buffSize;
  bsize = (int)buffSize;
  err = 0;
}

void* TComBitstream::getBitStreamBuffer() {
  return addr;
}

void TComBufferChunk::writeFinalCodeToBitstream(ofstream* outBitstream) {
	outBitstream->put(0);
	outBitstream->put(0);
	outBitstream->put(1);
	outBitstream->put(sequence_end_code);
}

void TComBufferChunk::writeStartCodeToBitstream(ofstream* outBitstream) {
	switch (getBufferType()) {
		case BufferChunkType(BCT_SPS):
			outBitstream->put(sequence_start_code);
			break;
		case BufferChunkType(BCT_MESH):
			outBitstream->put(mesh_payload_start_code);
			break;
		case BufferChunkType(BCT_GEOM):
			outBitstream->put(geometry_payload_start_code);
			break;
		case BufferChunkType(BCT_TEXTURE):
			outBitstream->put(texture_payload_start_code);
			break;
		default:
			break;
	}
}

void TComBufferChunk::writeToBitstream(ofstream* outBitstream, uint64_t length) {
	///< write start code
	outBitstream->put(0);
	outBitstream->put(0);
	outBitstream->put(1);

	///< write buffer start code
	writeStartCodeToBitstream(outBitstream);

	///< write buffer chunk data
	outBitstream->write((char*)getBitStreamBuffer(), length);
}

BufferChunkType TComBufferChunk::determineBufferType(const uint32_t& startCode) {
	switch (startCode) {
	case sequence_start_code:
		return BufferChunkType(BCT_SPS);
		break;
	case mesh_payload_start_code:
		return BufferChunkType(BCT_MESH);
		break;
	case geometry_payload_start_code:
		return BufferChunkType(BCT_GEOM);
		break;
	case texture_payload_start_code:
		return BufferChunkType(BCT_TEXTURE);
		break;
	case sequence_end_code:
		return BufferChunkType(BCT_MAX);
		break;
	default:
		return BufferChunkType(BCT_MIN);
		break;
	}
	return BufferChunkType(BCT_MIN);
}

int TComBufferChunk::readFromBitstream(ifstream& inBitstream, int buffersize, bool& decodeSequence, uint8_t& nextStartCode) {
	///< read start code 00 00 01
	if (!checkCond(inBitstream.get() == 0x00, "Error: invalid start code!"))
		return EXIT_FAILURE;
	if (!checkCond(inBitstream.get() == 0x00, "Error: invalid start code!"))
		return EXIT_FAILURE;
	if (!checkCond(inBitstream.get() == 0x01, "Error: invalid start code!"))
		return EXIT_FAILURE;

	uint8_t startCode = inBitstream.get();

	m_bufferType = determineBufferType(startCode);
	nextStartCode = 0;
	if (startCode == sequence_end_code) {
		decodeSequence = true;
		return EXIT_SUCCESS;
	}

	if (m_bufferType == BCT_MESH || m_bufferType == BCT_TEXTURE) {
		// read 4 Bytes length
		uint32_t payloadSize = 0;
		inBitstream.read(reinterpret_cast<char*>(&payloadSize), sizeof(payloadSize));
		allocateBuffSize(payloadSize);
		inBitstream.read(reinterpret_cast<char*>(addr), payloadSize);

		return EXIT_SUCCESS;
	}

	if (m_bufferType == BCT_SPS)
		allocateBuffSize(MAX_HEADER_BS_BUF);
	else
		allocateBuffSize(buffersize);

	size_t length = 0;
	if (readBufferChunk(inBitstream, length, nextStartCode) ==
		EXIT_FAILURE)  ///< read buffer chunk data
		return EXIT_FAILURE;
	// ——— 5) 去仿冒（EPB Removal）——把 0x03 等防仿冒字节去掉 
	length = initParsingConvertPayloadToRBSP(length, (unsigned char*)addr, (unsigned char*)addr2);  ///< demulate
	allocateBuffSize(length);
	return EXIT_SUCCESS;
}


TComBufferChunk::TComBufferChunk(BufferChunkType bufferType) {
	m_bufferType = bufferType;
}

void TComBufferChunk::setBufferType(BufferChunkType bufferType) {
	m_bufferType = bufferType;
}

BufferChunkType TComBufferChunk::getBufferType() {
	return m_bufferType;
}

int TComBufferChunk::readBufferChunk(ifstream& inBitstream, size_t& bufferChunkSize,
	uint8_t& nextStartCode) {
	unsigned char* pucBuffer = (unsigned char*)getBitStreamBuffer();
	unsigned char ucByte = 0;
	char bEndOfStream = 0;

	int iNextStartCodeBytes = 0;
	unsigned int iBytesRead = 0;
	unsigned int uiZeros = 0;
	unsigned char pucBuffer_Temp[16];
	int iBytesRead_Temp = 0;

	while (true) {
		inBitstream.read((char*)&ucByte, 1);

		if (inBitstream.eof()) {
			iNextStartCodeBytes = 0;
			bEndOfStream = 1;
			break;
		}
		pucBuffer[iBytesRead++] = ucByte;
		if (ucByte > 1)  ///< ucByte != 0 && UChar != 1
			uiZeros = 0;
		else if (ucByte == 0)  ///< 00
			uiZeros++;
		else if (uiZeros > 1)  ///< 00 00
		{
			iBytesRead_Temp = 0;
			pucBuffer_Temp[iBytesRead_Temp] = ucByte;

			iBytesRead_Temp++;
			inBitstream.read((char*)&ucByte, 1);

			pucBuffer_Temp[iBytesRead_Temp] = ucByte;
			pucBuffer[iBytesRead++] = ucByte;
			iBytesRead_Temp++;

			if (pucBuffer_Temp[0] == 0x01 &&
				(pucBuffer_Temp[1] >= start_code_lower &&
					pucBuffer_Temp[1] <= start_code_upper))  ///< 00 00 01 CODE
			{
				iNextStartCodeBytes = 2 + 1 + 1;  ///< encounter the next start code
				nextStartCode = pucBuffer_Temp[1];
				uiZeros = 0;
				break;
			}
			else {
				uiZeros = 0;
				iNextStartCodeBytes = 0;
			}
		}
		else {
			uiZeros = 0;
		}
	}
	bufferChunkSize = iBytesRead - iNextStartCodeBytes;

	if (bEndOfStream)
		return EXIT_SUCCESS;

	inBitstream.seekg(-1 * iNextStartCodeBytes,
		std::ios_base::cur);  ///< go back four bytes of start code
	return EXIT_SUCCESS;
}

size_t TComBufferChunk::initParsingConvertPayloadToRBSP(const size_t uiBytesRead, unsigned char* pBuffer,
	unsigned char* pBuffer2) {
	unsigned int uiZeroCount = 0;
	unsigned int uiBytesReadOffset = 0;
	unsigned int uiBitsReadOffset = 0;
	const unsigned char* pucRead = pBuffer;
	unsigned char* pucWrite = pBuffer2;
	unsigned int uiWriteOffset = uiBytesReadOffset;
	unsigned char ucCurByte = pucRead[uiBytesReadOffset];

	for (uiBytesReadOffset = 0; uiBytesReadOffset < uiBytesRead; uiBytesReadOffset++) {
		ucCurByte = pucRead[uiBytesReadOffset];
		if (2 <= uiZeroCount && 0x02 == pucRead[uiBytesReadOffset]) {
			pucWrite[uiWriteOffset] = ((pucRead[uiBytesReadOffset] >> 2) << (uiBitsReadOffset + 2));
			uiBitsReadOffset += 2;
			uiZeroCount = 0;
			if (uiBitsReadOffset >= 8) {
				uiBitsReadOffset = 0;
				continue;
			}
			if (uiBytesReadOffset >= uiBytesRead) {
				break;
			}
		}
		else if (2 <= uiZeroCount && 0x01 == pucRead[uiBytesReadOffset]) {
			uiBitsReadOffset = 0;
			pucWrite[uiWriteOffset] = pucRead[uiBytesReadOffset];
		}
		else {
			pucWrite[uiWriteOffset] = (pucRead[uiBytesReadOffset] << uiBitsReadOffset);
		}

		if (uiBytesReadOffset + 1 < uiBytesRead) {
			pucWrite[uiWriteOffset] |= (pucRead[uiBytesReadOffset + 1] >> (8 - uiBitsReadOffset));
		}
		uiWriteOffset++;

		if (0x00 == ucCurByte) {
			uiZeroCount++;
		}
		else {
			uiZeroCount = 0;
		}
	}

	// th just clear the remaining bits in the buffer
	for (unsigned int ui = uiWriteOffset; ui < uiBytesRead; ui++) {
		pucWrite[ui] = 0;
	}
	memcpy(pBuffer, pBuffer2, uiWriteOffset);
	return uiBytesRead;
}
//////////////////////////////////////////////////////////////////////////
// Private class functions
//////////////////////////////////////////////////////////////////////////

void TComBitstream::init() {
  addr = addr2 = pddr = nullptr;
  bsize = ssize = err = 0;
  memset(ndata, 0, 4 * sizeof(int));
  memset(pdata, 0, 4 * sizeof(void*));
  memset(ts, 0, sizeof(long long));
}

///< \}
