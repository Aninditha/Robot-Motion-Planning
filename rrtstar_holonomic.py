#!/usr/bin/env python

'''
Name: Aninditha Madishetty
ID #: 800936606
Email: amadishe@uncc.edu

Project title: Holonomic motion planning using RRT* Algorithm and collision 
'''

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2

# required variables
XLAST = 800
YLAST = 600
WINSIZE = [XLAST, YLAST]
EPSILON = 7.0
NUMNODES = 5000
RADIUS = 15
BLACK = (0, 0, 0)
Background = (255, 255, 255)
objectColor = (255, 0, 0)
color1 =  (91, 203, 120)
color2 = (255, 51, 116)
color3 = (155, 61, 193)
color4 = (61, 65, 193)
color5 = (255, 195, 0)
color6 = (88, 24, 69)
color7 = (172, 96, 36)

# node class
class Node:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord

#circle class
class Circle:
    x = 0
    y = 0
    radius = 0
    def __init__(self, x, y, radius):
	self.x = x
	self.y = y
	self.radius = radius

#ellipse class
class Ellipse:
    x = 0
    y = 0
    width = 0
    height = 0
    def __init__(self, x, y, width, height):
	self.x = x
	self.y = y
	self.width = width/2
	self.height = height/2

#rectangle class
class Rectangle:
    x = 0
    y = 0
    width = 0
    height = 0
    def __init__(self, x, y, width, height):
	self.x = x
	self.y = y
	self.width = width
	self.height = height

#triangle class
class Triangle:
    v1_x = 0
    v1_y = 0
    v2_x = 0
    v2_y = 0
    v3_x = 0
    v3_y = 0
    def __init__(self, v1_x, v1_y, v2_x, v2_y, v3_x, v3_y):
	self.v1_x = v1_x
	self.v1_y = v1_y
	self.v2_x = v2_x
	self.v2_y = v2_y
	self.v3_x = v3_x
	self.v3_y = v3_y

# calculates distance between two points
def distance(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def goto(p1,p2):
    if distance(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

#checks if a node is in the circle
def in_circle(node, circle):
    distance = math.sqrt((circle.x - node.x) ** 2 + (circle.y - node.y) ** 2)
    return distance <= circle.radius

#checks if a node is in the rectangle
def in_rectangle(node, rectangle):
    if((rectangle.x <= node.x  and node.x <= (rectangle.x + rectangle.width)) and (rectangle.y < node.y and node.y < (rectangle.y + rectangle.height))):
	return True
    else:
	return False

def sign (p1_x, p1_y, p2_x, p2_y, p3_x, p3_y):
    return (p1_x - p3_x) * (p2_y - p3_y) - (p2_x - p3_x) * (p1_y - p3_y)

#checks if a node is in the triangle
def in_triangle(node, triangle):
    b1 = sign(node.x, node.y, triangle.v1_x, triangle.v1_y, triangle.v2_x, triangle.v2_y) < 0.0
    b2 = sign(node.x, node.y, triangle.v2_x, triangle.v2_y, triangle.v3_x, triangle.v3_y) < 0.0
    b3 = sign(node.x, node.y, triangle.v3_x, triangle.v3_y, triangle.v1_x, triangle.v1_y) < 0.0
    return ((b1 == b2) and (b2 == b3))

#checks if a node is in the ellipse
def in_ellipse(node, ellipse):
    if ((((node.x - ellipse.x) * (node.x - ellipse.x)) / (ellipse.width * ellipse.width)) + (((node.y - ellipse.y) * (node.y - ellipse.y)) / (ellipse.height * ellipse.height)) <= 1):
        return True
    else:
        return False

#checks if a node suffers collision
def Collision(node, circle, rectangle, square, ellipse1, ellipse2, pentagon_rectangle, triangle1, triangle2, triangle3, triangle4, ptriangle1, ptriangle2, ptriangle3, ptriangle4):
    if(in_circle(node, circle) or in_rectangle(node, rectangle) or in_rectangle(node, square) or in_ellipse(node,ellipse1) or in_ellipse(node,ellipse2) or in_rectangle(node, pentagon_rectangle) or in_rectangle(node, triangle1) or in_triangle(node, triangle2) or in_rectangle(node, triangle3) or in_triangle(node, triangle4) or in_rectangle(node, ptriangle1) or in_triangle(node, ptriangle2) or in_rectangle(node, ptriangle3) or in_triangle(node, ptriangle4)):
        return False
    else:
        return True

def chooseParent(nn,newnode,nodes):
    for p in nodes:
	if distance([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+distance([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+distance([nn.x,nn.y],[newnode.x,newnode.y]):
	    nn = p
    newnode.cost=nn.cost+distance([nn.x,nn.y],[newnode.x,newnode.y])
    newnode.parent=nn
    return newnode,nn

def reWire(nodes,newnode,pygame,screen):
    white = 255, 255, 255
    black = 20, 20, 40
    for i in xrange(len(nodes)):
        p = nodes[i]
	if p!=newnode.parent and distance([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+distance([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
	    pygame.draw.line(screen,white,[p.x,p.y],[p.parent.x,p.parent.y])  
	    p.parent = newnode
            p.cost=newnode.cost+distance([p.x,p.y],[newnode.x,newnode.y]) 
            nodes[i]=p  
            pygame.draw.line(screen,black,[p.x,p.y],[newnode.x,newnode.y])                    
    return nodes

#draws a path from init to goal
def drawSolutionPath(start,goal,nodes,pygame,screen):
    pink = 200, 20, 240
    nn = nodes[0]
    for p in nodes:
	if distance([p.x,p.y],[goal.x,goal.y]) < distance([nn.x,nn.y],[goal.x,goal.y]):
	    nn = p
    while nn!=start:
	pygame.draw.line(screen,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],20)  
	nn=nn.parent
    nn = nodes[4998]
    #pygame.draw.line(screen,pink,[nn.x,nn.y],[goal.x,goal.y],20)
	
def main():
    #initialize and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)

    #adding caption and setting background color
    pygame.display.set_caption('RMP Project')
    screen.fill(Background)
    
    #creating obstacle objects
    circle = Circle(100, 450, 60)   
    rectangle = Rectangle(440, 90, 120, 220)
    square = Rectangle(640, 65, 120, 120)
    ellipse1 = Ellipse(375, 500, 220, 120)
    ellipse2 = Ellipse(725, 450, 120, 220)
    pentagon_rectangle = Rectangle(190, 240, 120, 100)
    triangle1 = Rectangle(40, 140, 220, 20)
    triangle2 = Triangle(160, 40, 160, 140, 260, 140)
    triangle3 = Rectangle(140, 40, 20, 100)
    triangle4 = Triangle(40, 140, 140, 140, 140, 40)
    ptriangle1 = Rectangle(190, 320, 120, 20)
    ptriangle2 = Triangle(190, 340, 240, 340, 240, 390)
    ptriangle3 = Rectangle(240, 340, 20, 100)
    ptriangle4 = Triangle(260, 340, 310, 340, 260, 390)
    pentagon_triangle = Triangle(200, 330, 300, 330, 250, 380)

    #drawing the obstacles on the 2D screen
    pygame.draw.circle(screen, color1, [100, 450], 50)
    pygame.draw.rect(screen, color2, [450, 100, 100, 200])
    pygame.draw.ellipse(screen, color3, [275, 450, 200, 100])
    pygame.draw.polygon(screen, color4, [[150, 50], [50, 150], [250, 150]])
    pygame.draw.polygon(screen, color5, [[200, 250], [300, 250], [300, 330], [250, 380], [200, 330]])
    pygame.draw.rect(screen, color6, [650, 75, 100, 100])
    pygame.draw.ellipse(screen, color7, [675, 350, 100, 200])

    #initial and goal positions of the object
    pygame.draw.rect(screen, objectColor, [0, 0, 20, 20])
    pygame.draw.rect(screen, objectColor, [XLAST-20, YLAST-20, 20, 20])

    #RRT* Algorithm implimentation
    nodes = []

    nodes.append(Node(10.0,10.0))
    start=nodes[0]
    goal=Node(XLAST-10,YLAST-10)
    for i in range(NUMNODES):
	while(True):
            rand = Node(random.random()*XLAST, random.random()*YLAST)
	    nn = nodes[0]
            for p in nodes:
	        if distance([p.x,p.y],[rand.x,rand.y]) < distance([nn.x,nn.y],[rand.x,rand.y]):
	            nn = p
            interpolatedNode= goto([nn.x,nn.y],[rand.x,rand.y])
	
	    newnode = Node(interpolatedNode[0],interpolatedNode[1])
	    #checking for collision
            if (Collision(newnode, circle, rectangle, square, ellipse1, ellipse2, pentagon_rectangle, triangle1, triangle2, triangle3, triangle4, ptriangle1, ptriangle2, ptriangle3, ptriangle4)):
                break

 	[newnode,nn]=chooseParent(nn,newnode,nodes)
       
	nodes.append(newnode)
	pygame.draw.line(screen,BLACK,[nn.x,nn.y],[newnode.x,newnode.y])
        nodes=reWire(nodes,newnode,pygame,screen)
        pygame.display.update()

        for e in pygame.event.get():
	   if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
	      sys.exit("Leaving because you requested it.")
    
    #draws the final path
    drawSolutionPath(start,goal,nodes,pygame,screen)
    pygame.display.update()

if __name__ == '__main__':
    
    main()
    running = True
    while running:
        for event in pygame.event.get():
	    if event.type == pygame.QUIT:
                running = False

