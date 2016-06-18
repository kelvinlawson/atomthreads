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

