import pygame
import csv
import math

# Initialize pygame
pygame.init()

# Set up the screen
screen_width, screen_height = 1920, 1080
screen = pygame.display.set_mode((screen_width, screen_height))

# Set up the colors
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PINK = (255, 0, 255)

# Set up the font
font = pygame.font.SysFont('Arial', 20)

# Set up the clock
clock = pygame.time.Clock()

x_offset = 1920 / 2
y_offset = 1080 / 2

# Function to plot a point
def plot_point(x, y, color=GREEN):
    pygame.draw.circle(screen, color, (int(x), int(y)), 3)

def rotate_point(x, y, angle):
    """Rotate a point counterclockwise by a given angle around the origin."""
    radians = math.radians(angle)
    rotated_x = x * math.cos(radians) - y * math.sin(radians)
    rotated_y = x * math.sin(radians) + y * math.cos(radians)
    return rotated_x, rotated_y

def sensor_position(xpos, ypos, theta_deg, sensor_distance):
    """Calculate the position of a wall based on sensor distance and orientation."""
    # Convert distance from mm to pixels if necessary
    sensor_distance_in_pixels = sensor_distance / 10 # Example conversion, adjust as needed

    

    # Calculate sensor reading position
    sensor_x, sensor_y = rotate_point(sensor_distance_in_pixels, 0, theta_deg)
    newXpos = xpos + sensor_x
    newYpos = ypos + sensor_y
    print(f"Sensor at {newXpos}, {newYpos}")
    return newXpos, newYpos

# Function to read and plot data from CSV
def plot_csv_data():
    with open('data.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Skip header row
        for row in csvreader:
            xpos, ypos, theta_deg, fsensor, lsensor, rsensor, robotdir = map(float, row)
            xpos, ypos = xpos + x_offset, ypos + y_offset
            plot_point(xpos, ypos)

            # Plot walls detected by sensors
            if fsensor > 0:  # Check if the sensor value is valid
                fx, fy = sensor_position(xpos, ypos, theta_deg, fsensor)
                #print(f"Front sensor at {fx}, {fy}")
                plot_point(fx, fy, RED)

            if lsensor > 0:
                #print(f"Left sensor at {lsensor}")
                lx, ly = sensor_position(xpos, ypos, theta_deg + 90, lsensor)
                plot_point(lx, ly, BLUE)

            if rsensor > 0:
                rx, ry = sensor_position(xpos, ypos, theta_deg - 90, rsensor)
                plot_point(rx, ry, PINK)


# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Clear the screen
    screen.fill(BLACK)

    # Plot data from the CSV
    plot_csv_data()

    # Update the display
    pygame.display.flip()

    # Cap the frame rate
    clock.tick(60)

pygame.quit()
