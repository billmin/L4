import os, sys


WEAK = 0
NEUTRA = 1
STRONG = 2

# interaction with entities on the ground
banned_interaction_with_other_entities = ['HIT', 'RUNOVER', 'RUNACROSS']

pedestrian = {'banned':['HIT'], 'ban_intensity':STRONG}
bicycle = {'banned':['HIT'], 'ban_intensity':STRONG}
tricyle = {'banned':['HIT'], 'ban_intensity':STRONG}
motorcycle = {'banned':['HIT'], 'ban_intensity':STRONG}
vehicle = {'banned':['HIT'], 'ban_intensity':STRONG}
fence = {'banned':['HIT'], 'ban_intensity':STRONG}
pole = {'banned':['HIT'], 'ban_intensity':STRONG}
building = {'banned':['HIT'], 'ban_intensity':STRONG}
barrel = {'banned':['HIT'], 'ban_intensity':STRONG}
cone = {'banned':['HIT'], 'ban_intensity':STRONG}
vegetation = {'banned':['RUNOVER'], 'ban_intensity':STRONG}
sidewalk = {'banned':['RUNOVER'], 'ban_intensity':STRONG}
white_solid = {'banned':['RUNACROSS'], 'ban_intensity':STRONG}
white_dot = {'banned':['RUNACROSS'], 'ban_intensity':NEUTRA}
yellow_solid = {'banned':['RUNACROSS'], 'ban_intensity':STRONG}
yellow_dot = {'banned':['RUNACROSS'], 'ban_intensity':NEUTRA}
double_yellow_solid = {'banned':['RUNACROSS'], 'ban_intensity':STRONG}
crosswalk = {'banned':['RUNOVER'], 'ban_intensity':NEUTRA}
stopline = {'banned':['RUNOVER'], 'ban_intensity':NEUTRA}
road = {'banned':['RUNOVER'], 'ban_intensity':WEAK}

# interaction with entities in the air

