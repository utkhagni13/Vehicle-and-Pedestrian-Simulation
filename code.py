import pygame
import sys
import random, time

# Initialize Pygame
pygame.init()
simulation1 = pygame.sprite.Group()
simulation2 = pygame.sprite.Group()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
LANE_WIDTH = 150
VEHICLE_MAX_VELOCITY = 8

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)


# Pedestrian class
class Pedestrian(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load('images/ped.png')

        # starting position of pedestrian
        self.x = SCREEN_WIDTH // 2 - self.image.get_rect().width // 2
        self.y = SCREEN_HEIGHT // 2 - LANE_WIDTH // 2 - 50

        # initial motivation of pedestrian to cross the road
        self.motivation = 1

        # add pedestrian to simulation
        simulation2.add(self)

    def move(self, vx, vy, vs):
        # update motivation
        
        # calc. navigational force

        # calc. vehicle's interaction with pedestrian

        self.y += 1  # Adjust speed as needed
        if self.y > SCREEN_HEIGHT:
            self.y = SCREEN_HEIGHT // 2 - LANE_WIDTH // 2 - 100  # Reset position when reaching the bottom


# Car class
class Car(pygame.sprite.Sprite):
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load('images/car.png')
        simulation1.add(self)
        self.x = 0
        self.y = SCREEN_HEIGHT // 2 - self.image.get_rect().height // 2
        self.velocity = VEHICLE_MAX_VELOCITY

    def move(self):
        self.velocity = random.randint(3,5)
        self.x += self.velocity  # Adjust speed as needed
        if self.x > SCREEN_WIDTH:
            self.x = 0  # Reset position when reaching the right edge


class Main:
    # Setting background image i.e. image of intersection
    background = pygame.image.load('images/lane.jpg')
    
    # Initialize the screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Pedestrian and Car Simulation")

    # Initialize objects
    car = Car()
    Pedestrian()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            screen.blit(background, (0, 0))

            ########################## update pedestrian's movement ##########################
            for pedestrian in simulation2:
                screen.blit(pedestrian.image, [pedestrian.x, pedestrian.y])
                # update movement of pedestrian
                pedestrian.move(car.x, car.y, car.velocity)

            ########################## update car's movement ##########################
            screen.blit(car.image, [car.x, car.y])
            car.move()
            # Check for pedestrian-car collision
            for ped in simulation2:
                if (
                    car.x < ped.x + ped.image.get_rect().width
                    and car.x + car.image.get_rect().width > ped.x
                    and car.y < ped.y + ped.image.get_rect().height
                    and car.y + car.image.get_rect().height > ped.y + ped.image.get_rect().height
                ):
                    print(f"car: {car.x} {car.y} {car.image.get_rect().width} {car.image.get_rect().height}, ped: {ped.x} {ped.y} {ped.image.get_rect().width} {ped.image.get_rect().height}")
                    print("Collision!")
                    sys.exit()
                    # add collision handling logic here.

            pygame.display.update()
            pygame.time.Clock().tick(60)  # Adjust the frame rate as needed
    
    except Exception as error:
        print("error:", error)
        sys.exit()
