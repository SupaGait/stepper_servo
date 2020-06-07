# Not used by the VS Code Debugger, only `cargo run`
target remote :3333
set print asm-demangle on

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break HardFault
break rust_begin_unwind

monitor arm semihosting enable

load
break main
continue
