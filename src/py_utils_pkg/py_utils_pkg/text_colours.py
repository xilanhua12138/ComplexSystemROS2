#!/usr/bin/env python3
################################################################################
# Filename: text_colours.py
# Created : 27 October 2018
# Author  : Derek Ripper
# Purpose : To provide colour formatting for text to screen for  the
#           HEARTS python programs
#
################################################################################
# Updates:
# 27 Dec 2021 Derek - Update print stmts for Python3
#
# 14 Feb 2019 Derek - added a "ToDo" colour
#                     also aded keyword before users txt for message type
#                     eg prt.debug now prints "DEBUG: <users text here>""
#
################################################################################
# usage:
#
#     import python_support_library.text_colours as Tc
#
#
#     prt =Tc.Tc()
#
#     prt.debug("My debug message")
#
#     prt.result("The answer is 42")
#
#     prt.red("print my text in red")
#
#     prt.bred("print my text in bold red"))
#
#################################################################################

class Tc(object):

    # misc ANSI screen conrols
    reset   = '\033[0m'      # reset screen defults for ALL termnal  text

    ph      = '{}'           # place holder used by format method in Python

    end     = ph+reset       # ends all ansi codes usage "ie reset to defaults"

    # text styles
    norm      = 'm'
    bold      = ';1m'
    faint     = ';2m'
    underline = ';4m'
    crossthru = ';8m'

    # Foreground colours
    fg_black   = '\033[30'
    fg_red     = '\033[31'
    fg_green   = '\033[32'
    fg_yellow  = '\033[33'
    fg_blue    = '\033[34'
    fg_magenta = '\033[35'
    fg_cyan    = '\033[36'
    fg_white   = '\033[37'


    # bacground colours
    bg_black   = '\033[40'
    bg_red     = '\033[41'
    bg_green   = '\033[42'
    bg_yellow  = '\033[43'
    bg_blue    = '\033[44'
    bg_magenta = '\033[45'
    bg_cyan    = '\033[46'
    bg_white   = '\033[47'



    # individual text styles by colour - NORMAL
    def red    (self,txt): print( Tc.fg_red    +Tc.norm+Tc.end.format(txt))
    def green  (self,txt): print( Tc.fg_green  +Tc.norm+Tc.end.format(txt))
    def yellow (self,txt): print( Tc.fg_yellow +Tc.norm+Tc.end.format(txt))
    def blue   (self,txt): print( Tc.fg_blue   +Tc.norm+Tc.end.format(txt))
    def magenta(self,txt): print( Tc.fg_magenta+Tc.norm+Tc.end.format(txt))
    def cyan   (self,txt): print( Tc.fg_cyan   +Tc.norm+Tc.end.format(txt))
    def white  (self,txt): print( Tc.fg_white  +Tc.norm+Tc.end.format(txt))

     # individul text styles by colour - BOLD
    def bred    (self,txt): print( Tc.fg_red    +Tc.bold +Tc.end.format(txt))
    def bgreen  (self,txt): print( Tc.fg_green  +Tc.bold +Tc.end.format(txt))
    def byellow (self,txt): print( Tc.fg_yellow +Tc.bold +Tc.end.format(txt))
    def bblue   (self,txt): print( Tc.fg_blue   +Tc.bold +Tc.end.format(txt))
    def bmagenta(self,txt): print( Tc.fg_magenta+Tc.bold +Tc.end.format(txt))
    def bcyan   (self,txt): print( Tc.fg_cyan   +Tc.bold +Tc.end.format(txt))
    def bwhite  (self,txt): print( Tc.fg_white  +Tc.bold +Tc.end.format(txt))


    # Standard text styles
    def debug   (self,txt): print( Tc.fg_white  +Tc.bold      +Tc.bg_blue+Tc.norm    + Tc.end.format("DEBUG  : "+txt))
    def input   (self,txt): print( Tc.fg_blue   +Tc.bold                             + Tc.end.format("INPUT  : "+txt))
    def warning (self,txt): print( Tc.fg_yellow +Tc.bold                             + Tc.end.format("WARNING: "+txt))
    def error   (self,txt): print( Tc.fg_red    +Tc.bold      +Tc.bg_white+Tc.norm   + Tc.end.format("ERROR  : "+txt))
    def info    (self,txt): print( Tc.fg_cyan   +Tc.bold                             + Tc.end.format("INFO   : "+txt))
    def result  (self,txt): print( Tc.fg_green  +Tc.bold                             + Tc.end.format("RESULT : "+txt))
    def todo    (self,txt): print( Tc.fg_yellow  +Tc.bold      +Tc.bg_blue+Tc.norm   + Tc.end.format("TODO   : "+txt))
    def blank   (self):     print("\n")

if __name__ == '__main__':
    import text_colours

    prt = text_colours.Tc()

    print("\nStandard Styles to be used for the purpose indicated .....")
    prt.debug  ("DEBUG  : This is the colour for temporary debug messages")
    prt.input  ("INPUT  : This is the colour for asking for input data")
    prt.warning("WARNING: This is the colour for non-fatal       messages")
    prt.error  ("ERROR  : This is the colour for     fatal error messages")
    prt.info   ("INFO   : This is the colour for useful Message/status info during processing")
    prt.result ("RESULT : This is the colour for significant results")
    prt.todo   ("ToDo   : This is the colour for yet todo items!")

    print("\nNormal Colours .....")
    prt.red     ("RED      : my text in RED")
    prt.green   ("GREEN    : my text in GREEN")
    prt.yellow  ("YELLOW   : my text in YELLOW")
    prt.blue    ("BLUE     : my text in BLUE")
    prt.magenta ("MAGENTA  : my text in MAGENTA")
    prt.cyan    ("CYAN     : my text in CYAN")
    prt.white   ("WHITE    : my text in WHITE")

    print("\nBold Colours .....")
    prt.bred    ("RED      : my text in BOLD  RED")
    prt.bgreen  ("GREEN    : my text in BOLD  GREEN")
    prt.byellow ("YELLOW   : my text in BOLD  YELLOW")
    prt.bblue   ("BLUE     : my text in BOLD  BLUE")
    prt.bmagenta("MAGENTA  : my text in BOLD  MAGENTA")
    prt.bcyan   ("CYAN     : my text in BOLD  CYAN")
    prt.bwhite  ("WHITE    : my text in BOLD  WHITE\n")
