/*
 * Copyright (c) 2012, Natie van Rooyen. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#ifndef _STDINT_H
typedef unsigned int		        uintptr_t ;
typedef int					        intptr_t ;
typedef unsigned long long		    uint64_t ;
typedef unsigned int		        uint32_t ;
typedef unsigned short		        uint16_t ;
typedef unsigned char		        uint8_t ;
typedef int					        int32_t ;
typedef short				        int16_t ;
typedef char				        int8_t ;
#endif

#ifndef OFFSETOF
#define OFFSETOF(TYPE, MEMBER) ((int) &((TYPE *)0)->MEMBER)
#endif

#ifndef INLINE
#define INLINE  __inline
#endif

/* IO definitions (access restrictions to peripheral registers) */
#ifndef __I
#define     __I     volatile           /*!< defines 'read only' permissions                 */
#endif
#ifndef __O
#define     __O     volatile             /*!< defines 'write only' permissions                */
#endif
#ifndef __IO
#define     __IO    volatile             /*!< defines 'read / write' permissions              */
#endif

#endif /* __TYPES_H__ */

