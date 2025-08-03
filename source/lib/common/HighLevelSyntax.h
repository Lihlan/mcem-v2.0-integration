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

struct SequenceParameterSet {
	uint32_t	 subdivFitSubdivIterCount = 0;

	//UInt profileId = 0;
	//UInt levelId = 0;
	//UInt frameRateCode = 0;
	//Bool geomRemoveDuplicateFlag = true;   ///< remove duplicate points (=1) or keep duplicate points (=0)
	//Bool attrPresentFlag = true;           ///< 0: without any attributes, 1: with at least one kind of attribute
	//UInt maxNumAttributesMinus1 = 1;       ///< maximum number of attribute categories
	//Bool multiAttributesSetFlag = 1;       ///<  0: indicate multi attribute is off, 1 indicate multi attribute is on.

	//Bool recolorMode = true;               ///< 0: regular recolor, 1: fast recolor
};

struct HighLevelSyntax {
	SequenceParameterSet sps;

};