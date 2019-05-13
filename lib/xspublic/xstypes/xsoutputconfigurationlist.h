
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSOUTPUTCONFIGURATIONLIST_H
#define XSOUTPUTCONFIGURATIONLIST_H

#include "xsoutputconfigurationarray.h"
#define XsOutputConfigurationList	XsOutputConfigurationArray

#ifndef __cplusplus
// obsolete:
#define XSOUTPUTCONFIGURATIONLIST_INITIALIZER	XSOUTPUTCONFIGURATIONARRAY_INITIALIZER
#define XsOutputConfigurationList_assign(thisPtr, size, src)	XsArray_assign(thisPtr, size, src)
#define XsOutputConfigurationList_construct(thisPtr, size, src)	XsOutputConfigurationArray_construct(thisPtr, size, src)
#define XsOutputConfigurationList_destruct(thisPtr)				XsArray_destruct(thisPtr)
#define XsOutputConfigurationList_copy(thisPtr, copy)			XsArray_copy(copy, thisPtr)
#define XsOutputConfigurationList_append(thisPtr, other)		XsArray_append(thisPtr, other)
#define XsOutputConfigurationList_popFront(thisPtr, count)		XsArray_erase(thisPtr, 0, count)
#define XsOutputConfigurationList_popBack(thisPtr, count)		XsArray_erase(thisPtr, (XsSize)-1, count)
#define XsOutputConfigurationList_swap(a, b)					XsArray_swap(a, b)
#define XsOutputConfigurationList_erase(thisPtr, index, count)	XsArray_erase(thisPtr, index, count)
#define XsOutputConfigurationList_equal(a, b)					XsArray_equal(a, b)
#endif

#endif
