#ifndef AUDIO_HPP
#define AUDIO_HPP

#include "functions.hpp"
#include <Arduino.h>

// Audio module definitions

// ----------------- MP3 Defaults Values ----------------
#define CMD_PLAY_FOLDER_TRACK 0x0F
// -> Tracks for button A pressed - > Main node
#define A_FOLDER_TRACK_A 0x0201
#define B_FOLDER_TRACK_A 0x0202
#define C_FOLDER_TRACK_A 0x0203
#define D_FOLDER_TRACK_A 0x0204
#define E_FOLDER_TRACK_A 0x0205
#define F_FOLDER_TRACK_A 0x0206
// -> Tracks for button B pressed - > bond
#define A_FOLDER_TRACK_B 0x0101
#define B_FOLDER_TRACK_B 0x0102
#define C_FOLDER_TRACK_B 0x0103
#define D_FOLDER_TRACK_B 0x0104
#define E_FOLDER_TRACK_B 0x0105
#define F_FOLDER_TRACK_B 0x0106

// ------------------- FUNCTIONS ---------------------

/// @brief Returns the audio track associated with the node-button pair pressed
/// @param node_id The node that was pressed
/// @param button_pressed The button of the node pressed
/// @return Returns the audio-track folder combination to pass to the mp3 module
int getNodeAudioTrack(int node_id, uint8_t button_pressed);

#endif