ENTRY(start);

SECTIONS {
	. = 0x0100;

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
	bsswords = (end - edata) / 2;	
}
