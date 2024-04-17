#!/usr/bin/env python3
################################################################################
# Filename: alsa_audio.py
# Author  : Derek Ripper
# Created : 05 Oct 2022
# Purpose : Define a class to switch speakers/mic ON & OFF as needed
#                 Required by TTS to stop robot listening to its self!
################################################################################
# Updates:
# ??/???/???? by ????? - info ......
################################################################################
import alsaaudio
class Audio_control():
    def __init__(self):
        self.mixer_spk = alsaaudio.Mixer(control='Master',  id=0)
        self.mixer_mic = alsaaudio.Mixer(control='Capture', id=0)
        self.muted     = 1 # muted
        self.notmuted  = 0 # not muted
        self.micoff    = 0 # mic off
        self.micon     = 1 # mic on
        
        self.mic_on()      # ensure intial status of: mic "on" and speaker "off"
        
    def list_status(self):
        print("\n***** ALSAAUDIO current ssettings for MIC & SPK")
        ans = self.mixer_mic.getenum()
        print("*** mixer.eum()")
        print(ans)
        print("*** mixer.mixerid()")
        ans = self.mixer_mic.mixerid()
        print(ans)
        print("*** mixer.switchcap")
        
        ans = self.mixer_mic.switchcap()
        print(ans)
        
        print("\n*** mixer.getmute()")
        spk_status = self.mixer_spk.getmute()
        ans = spk_status[0]
        if ans== 1:
            print("* Spk is INActive")
        elif ans == 0:
            print("* Spk is Active")
        else:
            print("* Spk Status has an unknown value = "+str(ans))
        
        print("*** mixer.getrec()")
        mic_status = self.mixer_mic.getrec()
        ans = mic_status[0]
        if ans == 1:
            print("* Mic is Active")
        elif ans == 0:
            print("* Mic is INActive")
        else:
            print("* Mic Status has an unknown value = "+str(ans))
  
    def set_spk(self,arg1):
        self.mixer_spk.setmute(arg1)
   
    def set_mic(self,arg1):
        self.mixer_mic.setrec(arg1)

    def mic_off(self):
        self.set_mic(self.micoff)
        self.set_spk(self.notmuted)

    def mic_on(self):
        self.set_spk(self.muted)
        self.set_mic(self.micon)
        
        
##### end of class def for "Audio_control"

def main(args=None):
    d = Audio_control()
    d.mic_off()       ## mic     OFF, Speaker ON
    d.mic_on()        ## speaker OFF, mic  ON
    d.list_status()
    print("\n*****  The End  *****")

if  __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        prt.blank()

        prt.warning(cname+" Cancelelld by user !")
	
