#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



import hubo_ach as ha
import ach
import sys
import time
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r_in = ach.Channel(ha.HUBO_CHAN_REF_FILTER_NAME)
r_out = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref_in = ha.HUBO_REF()
ref_out = ha.HUBO_REF()

# filter length
filtLength = ha.HUBO_REF_FILTER_LENGTH * 2.0  

# sleep time (sec)
sleepTime = 0.005

def doRefFilter(ref, state, L):
  out = (ref * (L-1.0) + ref) / L;
  return out

if __name__=='__main__':

  # Get the current feed-forward (state) 
  while(True):
    [statuss, framesizes] = s.get(state, wait=False, last=True)
    [statusr_in, framesizer_in] = r_in.get(ref_in, wait=False, last=True)

    # filter reference
    for i in range(0, ha.HUBO_JOINT_COUNT):
      ref_out.ref[i] = doRefFilter(ref_in.ref[i], state.joint[i].pos, filtLength)

    #push on ref channel
    r_out.put(ref_out)

    # sleep (don't care much for RT)
    time.sleep(sleepTime)


  # Close the connection to the channels
  r_in.close()
  r_out.close()
  s.close()

