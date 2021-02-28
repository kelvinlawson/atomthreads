; Copyright (c) 2016 Dr. Philipp Klaus Krause

; Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

; The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

.area CODE

; uint8_t get_cc(void);
_get_cc::
	push	cc
	pop	a
	ret

; void set_cc(uint8_t);
_set_cc::
	ld	a, (3, sp)
	push	a
	pop	cc
	ret

; void archContextSwitch (ATOM_TCB *old_tcb_ptr, ATOM_TCB *new_tcb_ptr)
_archContextSwitch::

	; save context
	ldw	x, (3, sp)
	ldw	y, sp
	ldw	(x), y

	; restore context
	ldw	x, (5, sp)
	ldw	x, (x)
	ldw	sp, x

	ret

; void archFirstThreadRestore (ATOM_TCB *new_tcb_ptr)
_archFirstThreadRestore::

	; restore context
	ldw	x, (3, sp)
	ldw	x, (x)
	ldw	sp, x

	ret

