drc_hubo_master_slave
=====================

<p>This program is to control drc-hubo through a mini-hubo</p>

<p>1)Hubo-ACH</p>
<p>   https://github.com/hubo/hubo-ach</p>

<p>2) getch</p>
<p>   https://pypi.python.org/packages/source/g/getch/getch-1.0-python2.tar.gz</p>

<p>3) pydynamixel </p>
<p>   git clone https://github.com/thedancomplex/pydynamixel </p>


<p> you need openhubo and openrave installed</p>

<p> in the mappings.py (if you are using the hubo-ach install sim-all version) you need to add</p>

<p> 'NK3':'NKY' </p>

<p> to the synonyms in order to get all of the proper joints </p>

<p> NOTE: THIS HASN'T BEEN TRIED WITHOUT THAT CHANGE SO IT MAY WORK.....it was merely thought that having all of the joints maping properly would be a good thing </p>

<p> you need both the filter and the master_slave program running </p>
