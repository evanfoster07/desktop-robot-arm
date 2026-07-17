#pragma once

/*
    Initializes any state required by the command handler.
*/
void beginSerialCommands();

/*
    Reads available characters from the PC serial connection.
*/
void readSerialCommands();