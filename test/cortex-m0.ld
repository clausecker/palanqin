ENTRY(_start);

SECTIONS {
	. = 0x0;

	.text : {
		*(.text.startup);
		*(.text);
	}

	etext = .;

	.data : {
		*(.data);
		*(.rodata);
		. = ALIGN(2);
	}

	edata = .;

	.bss : {
		*(.bss);
	}

	end = .;
}
