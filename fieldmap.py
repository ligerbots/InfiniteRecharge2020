#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt

# Work in inches since we have those easily

field_height = 26.0 * 12 + 11.25    # 8.21 m
field_width = 52.0 * 12 + 5.25      # 15.98 m
initiate_line = 10*12.0
ball_rad = 3.5

trench_h = 56.0
trench_w = 18.0 * 12.0


# there are always matching balls, reflected across the center
# so draw a pair
def draw_ball(x, y):
    c = plt.Circle((x, y), ball_rad, fill=True, color='yellow')
    plt.gcf().gca().add_artist(c)
    c = plt.Circle((field_width - x, field_height - y), ball_rad, fill=True, color='yellow')
    plt.gcf().gca().add_artist(c)
    return


# make sure to set the x and y directions to be equal
fig1, axis1 = plt.subplots()
axis1.set_aspect('equal')

axis1.set_xlim((0, field_width))
axis1.set_ylim((0, field_height))

# general outline. Ignore the angled driver stations for now
plt.plot((0, 0, field_width, field_width, 0), (0, field_height, field_height, 0, 0), 'green')
plt.plot((initiate_line, initiate_line), (0, field_height), 'blue')
plt.plot((field_width - initiate_line, field_width - initiate_line), (0, field_height), 'red')

# shield generator
sg_width = 13.0 * 12 + 1.5
sg_depth = 14.0 * 12 + 0.75
sg_leg_size = 12.0
sg_rotation = 22.5  # degrees

# target zone
target_h = 4 * 12.0
target_w = 28.0
target_yc = 94.905          # from onshape
plt.plot((field_width, field_width - target_w, field_width),
         (target_yc - target_h/2, target_yc, target_yc + target_h/2), 'blue')
plt.plot((0, target_w, 0),
         (field_height - target_yc - target_h/2, field_height - target_yc, field_height - target_yc + target_h/2),
         'red')

# draw the trenches
x = (field_width - trench_w) / 2
x2 = x + trench_w
plt.plot((x, x, x2, x2), (0, trench_h, trench_h, 0), 'blue')
plt.plot((x, x, x2, x2), (field_height, field_height - trench_h, field_height - trench_h, field_height), 'red')

# trench balls
for i in range(3):
    draw_ball(field_width / 2.0 + 36.0 * i, trench_h / 2.0)
y = 19.054                      # from onshape
x = 128.405 + initiate_line     # from onshape
draw_ball(x, y)
y += 18.5
draw_ball(x, y)

#shield generator (based on https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf)
shield_generator_points=[(198.75,field_height/2+43.75),(198.75+116+51.06,field_height/2+112.88)]
shield_generator_balls=[(130.25,19.79),(114.94,26.13),(107.83, 50.54),(114.17,65.84),(120.51, 81.14)]
shield_generator_balls=[(field_width-120-x,y+ 94.66) for x,y in shield_generator_balls]

for point in shield_generator_balls:
    draw_ball(*point)

plt.plot(*zip(*(
    shield_generator_points +
    [(field_width-x,field_height-y) for x,y in shield_generator_points] +
    shield_generator_points[0:1]
)),"gray")



plt.axis('off')
plt.savefig("fieldmap.png", bbox_inches='tight', pad_inches=0, dpi=400, transparent=True)
plt.show()
