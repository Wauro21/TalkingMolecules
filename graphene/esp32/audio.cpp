#include "audio.hpp"

int getNodeAudioTrack(int node_id, uint8_t button_pressed)
{
    switch (node_id)
    {
    case 8:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return A_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return A_FOLDER_TRACK_B;
        }
        break;
    }
    case 9:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return B_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return B_FOLDER_TRACK_B;
        }
        break;
    }
    case 10:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return C_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return C_FOLDER_TRACK_B;
        }
        break;
    }
    case 11:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return D_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return D_FOLDER_TRACK_B;
        }
        break;
    }
    case 12:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return E_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return E_FOLDER_TRACK_B;
        }
        break;
    }
    case 13:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return F_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return F_FOLDER_TRACK_B;
        }
        break;
    }

    default:
        return 0;
    }
}