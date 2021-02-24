/**
 * @file color.h
 * @brief Defines all of the ANSI terminal escape codes that modify the color of text.
 */

#ifndef COLOR_H
#define COLOR_H

#define COLOR_RESET  "\033[0m"
#define BOLD         "\033[1m"
#define BLACK_TEXT   "\033[30;1m"
#define RED_TEXT     "\033[31;1m"
#define GREEN_TEXT   "\033[32;1m"
#define YELLOW_TEXT  "\033[33;1m"
#define BLUE_TEXT    "\033[34;1m"
#define MAGENTA_TEXT "\033[35;1m"
#define CYAN_TEXT    "\033[36;1m"
#define WHITE_TEXT   "\033[37;1m"

// For use with
// fprintf(stdout, "Recognized text: %s\n", text ? text : (RED_TEXT "No text recognized." COLOR_RESET));
// fprintf(stdout, (RED_TEXT "%s :: bool share_ckf(hast::ckfshare::Request &req, hast::ckfshare::Response &res)\n" COLOR_RESET), ugv.s_data_filename.c_str());


#endif // COLOR_H
