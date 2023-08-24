# BL618_terminal
This repository includes popular drivers of peripherals on BL618 and command handlers.



## folder hierarchy

-- peripherals

 |- sensor: drivers of some popular sensors

 |- actuator: drivers of motors and servos

-- command.h & command.c: handle commands and provide a standard output middleware

-- final_board_config.c & final_board_config.h: config ports



## usage

1. Clone [bouffalo_sdk](https://github.com/bouffalolab/bouffalo_sdk)
2. Find an example suitable for your project
3. Copy drivers you need into your project and add build configuration into CMakeLists.txt
4. If you need command handler, copy command.c & command.h into your project, add command_line_nonprint or command_line to deal with commands and the result is in print_lines and length is in print_lines_pos

