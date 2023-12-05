import pygame
import numpy as np
import sys, random, math, threading, time

# Initialize Pygame
pygame.init()
simulation1 = pygame.sprite.Group()
simulation2 = pygame.sprite.Group()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
LANE_WIDTH = 150
VEHICLE_LENGTH = 80
VEHICLE_WIDTH = 30
VEHICLE_MAX_VELOCITY = 8.0
PEDESTRIAN_MAX_VELOCITY = 1.0
VEHICLE_MAX_ACC = 3.0

# pedestrian constants
alpha = 0.8
psi = np.array([3, -0.3])
beta = 2.2
t_reaction = 0.05
theta = 0.3

kd = 200.0
sigma_nav = 0.09

A_shape = 800.0
d0_shape = 4.0
sigma_shape = 0.1

A_flow = 600.0
d0_flow = 6.0
sigma_flow = 0.1

A_speed = 400.0
delT = 1.0
sigma_y = 0.2 * LANE_WIDTH

mass = 75.0
kv = 0.1

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# miscellaneous functions
def magnitude(vec):
    return math.sqrt(vec[0]**2 + vec[1]**2)


# Pedestrian class
class Pedestrian(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load('images/ped.png')

        # starting position of pedestrian
        x = SCREEN_WIDTH // 2 - self.image.get_rect().width // 2
        y = SCREEN_HEIGHT // 2 - LANE_WIDTH // 2 - 50
        self.p0 = np.array([x, y])
        self.pos = np.array([x, y])
        
        gx = SCREEN_WIDTH // 2 - self.image.get_rect().width // 2
        gy = SCREEN_HEIGHT // 2 + LANE_WIDTH // 2 + 5
        self.goal = np.array([gx, gy])
        self.initial_to_goal_vec = (self.goal - self.p0)
        self.initial_to_goal_mag = magnitude(self.goal - self.p0)
        
        self.curr_vel = np.array([0, PEDESTRIAN_MAX_VELOCITY])
        self.desr_vel = self.curr_vel

        self.acceleration = np.array([0, 0])

        # initial motivation of pedestrian to cross the road
        self.motivation = 1

        # add pedestrian to simulation
        simulation2.add(self)
    
    def updateMotivation(self, D_pv, cv, ca):
        t_adv = (D_pv/cv) - (LANE_WIDTH/magnitude(self.curr_vel)) - t_reaction
        f = np.array([t_adv, ca])

        f.transpose()
        p = math.exp(-(psi.dot(f) - beta))
        M_bar = 1/(1 + p)
        self.motivation = alpha * self.motivation + (1 - alpha) * M_bar
    
    def getH(self, d, A, d0, sigma):
        return A/(2 * d0) * (d0 - d + math.sqrt( (d0 - d)**2 + sigma**2 ))

    def move(self, car_pos, car_vel, car_acc):
        # update motivation
        cv, ca = magnitude(car_vel), magnitude(car_acc)
        D_pv = abs(self.pos[0] - (car_pos[0] + VEHICLE_LENGTH))
        self.updateMotivation(D_pv, cv, ca)
        
        # calc. navigational force
        v_des = (magnitude(self.desr_vel)/math.sqrt( magnitude(self.goal - self.pos)**2 + sigma_nav**2 )) * (self.goal - self.pos)
        F_nav = (self.motivation * kd) * (self.curr_vel - v_des)
        print(f"D_pv: {D_pv}, Motivation: {self.motivation}, v_des: {v_des}, F_nav: {F_nav[1]}\n")

        # vehicle's influence on pedestrian
        # calc. F_shape
        a, b = VEHICLE_LENGTH/2, VEHICLE_WIDTH/2
        x, y = self.pos[0] - car_pos[0], self.pos[1] - car_pos[1]
        print(x, y)
        d = math.sqrt( (x/a)**2 + (y/b)**2 )
        h_shape = self.getH(d, A_shape, d0_shape, sigma_shape)
        shape_const = np.array([ (2*x)/(a**2), (2*y)/(b**2) ])
        F_shape = h_shape/magnitude(shape_const) * shape_const
        print(f"Hs: {h_shape}, F_shape: {F_shape}\n")

        # calc. F_flow
        P = ( (self.pos - self.p0).dot(self.initial_to_goal_vec) / self.initial_to_goal_mag ) # projection of curr pos on direct path
        kf_p = 1.0
        if (P <= 0):
            kf_p = 1.0
        elif (P > self.initial_to_goal_mag):
            kf_p = 0
        else:
            kf_p = (self.initial_to_goal_mag - P)/self.initial_to_goal_mag
        
        h_flow = self.getH(d, A_flow, d0_flow, sigma_flow)
        flow_const = np.array([ (-2 * (y**3))/b, (2 * (x**3))/a ])
        F_flow = (kf_p * h_flow)/magnitude(flow_const) * flow_const
        print(f"P: {P}, Kf_p: {kf_p}, Hf: {h_flow}, F_flow: {F_flow}\n")

        # calc. F_speed
        F_speed = np.array([ 0, A_speed * math.exp(-(x - a)/(cv * delT)) * math.exp(-(y**2)/(2 * sigma_y**2)) ])
        print(f"F_speed: {F_speed}\n")

        # calc. F_veh
        kv_param = 1/(1 + kv * cv**2)
        F_veh = F_shape + (kv_param) * F_flow + (1 - kv_param) * F_speed
        print(f"F_veh: {F_veh}\n")

        # calc. F_total
        F_total = F_nav + F_veh
        self.acceleration = F_total/mass
        print(f"F_total: {F_total}, acc: {self.acceleration}, dist: {self.goal - self.pos}")

        # move the pedestrian
        # print(f"vel: {self.curr_vel}, {}")
        self.curr_vel[0] = math.sqrt(abs(self.curr_vel[0]**2 + 2 * (self.goal - self.pos)[0] * self.acceleration[0]))
        self.curr_vel[1] = math.sqrt(abs(self.curr_vel[1]**2 + 2 * (self.goal - self.pos)[1] * self.acceleration[1]))
        self.pos = self.pos + self.curr_vel
        print(f"vel: {self.curr_vel}, position: {self.pos}")
        if self.pos[1] > SCREEN_HEIGHT:
            self.pos[1] = SCREEN_HEIGHT // 2 - LANE_WIDTH // 2 - 100  # Reset position when reaching the bottom
        print("______________________________________________________________________________________________________________")
        print()
        print()


# Car class
class Car(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load('images/car.png')

        self.pos = np.array([0, SCREEN_HEIGHT // 2 - VEHICLE_WIDTH // 2])

        self.curr_vel = np.array([VEHICLE_MAX_VELOCITY, 0])
        self.acceleration = np.array([0, 0])
        
        simulation1.add(self)

    def move(self):
        self.curr_vel[0] = random.randint(5, VEHICLE_MAX_VELOCITY)
        self.pos[0] += self.curr_vel[0]  # Adjust speed as needed
        if self.pos[0] > SCREEN_WIDTH:
            self.pos[0] = 0  # Reset position when reaching the right edge


def text_objects(text, font):
    textSurface = font.render(text, True, (0,0,0))
    return textSurface, textSurface.get_rect()

def paused(screen):
    largeText = pygame.font.SysFont("comicsansms", 55, bold=True)
    TextSurf, TextRect = text_objects("CRASH!!!", largeText)
    TextRect.center = ((SCREEN_WIDTH/2),(SCREEN_HEIGHT/20))
    screen.blit(TextSurf, TextRect)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        pygame.display.update()
        pygame.time.Clock().tick(60)


class Main:
    # Setting background image i.e. image of intersection
    background = pygame.image.load('images/lane.jpg')
    
    # Initialize the screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Pedestrian and Car Simulation")

    # Initialize objects
    car = Car()
    Pedestrian()
    global VEHICLE_LENGTH, VEHICLE_WIDTH
    VEHICLE_LENGTH = car.image.get_rect().width
    VEHICLE_WIDTH = car.image.get_rect().height
    timer = 0

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            screen.blit(background, (0, 0))

            cx, cy = car.pos[0], car.pos[1]
            ########################## update pedestrian's movement ##########################
            for pedestrian in simulation2:
                screen.blit(pedestrian.image, [pedestrian.pos[0], pedestrian.pos[1]])
                # update movement of pedestrian
                # if(timer % 1000 == 0):
                pedestrian.move(car.pos, car.curr_vel, car.acceleration)

            ########################## update car's movement ##########################
            screen.blit(car.image, [cx, cy])
            # if(timer % 1000 == 0):
            car.move()
            # Check for pedestrian-car collision
            for ped in simulation2:
                px, py = ped.pos[0], ped.pos[1]
                if (
                    cx < px + ped.image.get_rect().width
                    and cx + VEHICLE_LENGTH > px
                    and cy < py + ped.image.get_rect().height
                    and cy + VEHICLE_WIDTH > py + ped.image.get_rect().height
                ):
                    # print(f"car: {cx} {cy} {car.image.get_rect().width} {car.image.get_rect().height}, ped: {px} {py} {ped.image.get_rect().width} {ped.image.get_rect().height}")
                    print("COLLISION!!!")
                    paused(screen)
                    # add collision handling logic here.

            timer += 1
            pygame.display.update()
            pygame.time.Clock().tick(60)  # Adjust the frame rate as needed
    
    except Exception as error:
        print("error:", error)
        sys.exit()
