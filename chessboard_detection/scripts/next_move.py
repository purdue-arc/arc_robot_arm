import chess_engine
import image_processor

# Loads game from state file
def load_game(file):
    with open(file, 'r') as fo:
        fen_history = []
        for line in fo:
            fen_history.append(line.strip('\n'))
    state_fen = fen_history[-1]
    side = fen_history[0]
    game = chess_engine.ChessEngine(state_fen, side)
    return game

# Gets the coordinate positions of the robot's move
def get_positions(game):
    # Get move for the robot from chess AI
    robot_move = game.AI_move()
    print(f"Robot Move: {robot_move}")
    # Get the positions the robot needs to go to perform the move
    positions = game.convert_to_coordinates(robot_move)
    return positions

# Makes user's move on the game board given before and after images
def make_user_move(IMG_1, IMG_2, game):
    side = game.get_side()
    state_matrix = game.get_state_matrix()
    # Find change --> find move
    user_move = image_processor.get_move(IMG_1, IMG_2, side, state_matrix)
    print(f"User Move: {user_move}")
    # Make the move
    flag = game.make_move(user_move)
    return flag

# Updates the state file
def update_state(file, game):
    state_fen = game.get_state_fen()
    with open(file, 'a') as fo:
        fo.write(f'{state_fen}\n')
    return

def get_next_move(IMG_1, IMG_2, file):
    game = load_game(file)
    positions = [-1]
    if game.is_playable():
        if not game.is_robot_turn():
            flag = make_user_move(IMG_1, IMG_2, game)
            if not flag:
                print("Move not detected, try again.")
                return positions
        # Get positions corresponding to the robot's move
        positions = get_positions(game)
        # Store the state
        update_state(file, game)
    else:
        print("Game is unplayable")

    return positions
