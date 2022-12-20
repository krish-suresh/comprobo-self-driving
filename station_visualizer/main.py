import pygame
import button

# create display window
SCREEN_WIDTH, SCREEN_HEIGHT = 1400, 800
BLUE = (0, 41, 107)
YELLOW = (253, 197, 0)
WHITE = (255, 255, 255)

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Comprobo")

track_car_img = pygame.image.load("images/track_car.png").convert_alpha()
start_img = pygame.image.load("images/startbutton.png").convert_alpha()
stop_img = pygame.image.load("images/stop.png").convert_alpha()
leaderboard_img = pygame.image.load("images/leaderboard.png").convert_alpha()
car_img = pygame.image.load("images/car.png").convert_alpha()
border_img = pygame.image.load("images/border.png").convert_alpha()

home_background = pygame.image.load("images/home.jpg").convert_alpha()
home_background = pygame.transform.scale(home_background, (1400, 800))
visualizer_background = pygame.image.load("images/v_screen.jpg").convert_alpha()
visualizer_background = pygame.transform.scale(visualizer_background, (1400, 800))


# create button instances
track_button = button.Button(
    track_car_img.get_width() / 2 - 250,
    100 - track_car_img.get_height() / 2,
    track_car_img,
    1,
)
leaderboard_button = button.Button(
    leaderboard_img.get_width() / 2 + 700,
    100 - leaderboard_img.get_height() / 2,
    leaderboard_img,
    1,
)
start_button = button.Button(900, 200, start_img, 1)
stop_button = button.Button(900, 400, stop_img, 1)

# mark car's position

# menu
class Menu:
    def __init__(self):
        self.state = "home"

    def state_manager(self):
        if self.state == "home":
            self.home()
        elif self.state == "visualizer":
            self.visualizer()
        elif self.state == "leaderboard":
            self.leaderboard()

    def home(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                pygame.sys.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if track_button.rect.collidepoint(pygame.mouse.get_pos()):
                    self.state = "visualizer"
                if leaderboard_button.rect.collidepoint(pygame.mouse.get_pos()):
                    self.state = "leaderboard"

        pygame.display.update()
        screen.blit(home_background, (0, 0))
        track_button.draw(screen)
        leaderboard_button.draw(screen)

    def visualizer(self):
        # if start_button.draw(screen):
        #     print('START')
        # if stop_button.draw(screen):
        #     print('STOP')

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                pygame.sys.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if start_button.rect.collidepoint(pygame.mouse.get_pos()):
                    print("START")
                if stop_button.rect.collidepoint(pygame.mouse.get_pos()):
                    print("STOP")
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.state = "home"

        pygame.display.update()
        screen.blit(visualizer_background, (0, 0))
        start_button.draw(screen)
        stop_button.draw(screen)

    def leaderboard(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                pygame.sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.state = "home"
        pygame.display.update()
        screen.blit((visualizer_background), (0, 0))


menu = Menu()

running = True
while running:
    menu.state_manager()
