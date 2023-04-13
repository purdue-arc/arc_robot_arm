import cv2
import numpy as np

# Function returns most likely positions where a change occured given a matrix of change values
def get_locations_of_max_values(matrix, threshold):
    positions = []
    # Define cutoff for max values
    cutoff = np.max(matrix)*threshold
    for i in range(8):
        for j in range(8):
            # Add index of an element if its value is greater than the cutoff
            if matrix[i][j] > cutoff:
                positions.append([i, j])
    return positions

# Function that converts matrix coordinates to chess coordinates
def convert_to_chess_notation(coordinates, white):
    letter = {1: 'a', 2: 'b', 3: 'c', 4: 'd', 5: 'e', 6: 'f', 7: 'g', 8: 'h'}
    if white:
        return letter[coordinates[1]+1] + str(8-coordinates[0])
    return letter[8-coordinates[1]] + str(coordinates[0]+1)

# Function that takes in two images and returns the total change for each square on the chessboard
def get_change(image_1, image_2, dims, crop, display):

    # Corners of chessboard in the images
    UL, LR = crop

    # Image matrices of pixel values cropped to contain only the chessboard
    img_initial = cv2.imread(image_1)[UL[0]:LR[0], UL[1]:LR[1]]
    img_final = cv2.imread(image_2)[UL[0]:LR[0], UL[1]:LR[1]]

    # Resize the image
    img_initial = cv2.resize(img_initial, dims)
    img_final = cv2.resize(img_final, dims)

    # Normalize the image
    img_initial_normalized = cv2.normalize(img_initial, None, alpha = 0, beta = 1, norm_type = cv2.NORM_MINMAX)
    img_final_normalized = cv2.normalize(img_final, None, alpha = 0, beta = 1, norm_type = cv2.NORM_MINMAX)

    # Calculate the squared change and sum over the color channels to get a 2D matrix
    delta = np.sum((img_final_normalized-img_initial_normalized)**2, axis = 2)

    # Display change matrix
    if display:

        cv2.imshow("Initial Image", img_initial)
        cv2.waitKey(0)
        cv2.imshow("Final Image", img_final)
        cv2.waitKey(0)
        cv2.imshow("Change Image", delta/3)
        cv2.waitKey(0)

    # Dimensions of the change matrix in pixels
    rows, columns = delta.shape[0], delta.shape[1]

    # Dimensions of a single square in pixels
    single_square_height, single_square_length = int(rows/8), int(columns/8)

    # Matrix containing the cumulative change for each square
    accumulated_change = np.zeros((8, 8))

    # Loop over each square in the chessboard
    for i in range(8):
        for j in range(8):
            # Indices for a certian square in the change matrix
            start_row = i*single_square_height
            end_row = (i+1)*single_square_height
            start_column = j*single_square_length
            end_column = (j+1)*single_square_length
            # Slice the change matrix to focus on a certain square
            single_square = delta[start_row:end_row, start_column:end_column]

            ### ======== HEURISTICS TO PREVENT ISSUES FROM CHANGES IN LIGHTING ======== ###
            # Flatten the square to 1D array
            single_square_flattened = single_square.flatten()
            # Compute the mean change in the given square
            mean_change_value = np.mean(single_square)
            # Accumulate all the significant changes which make a clear contrast between initial and final values
            significant_change = 0
            for n in single_square_flattened:
                # Only consider change values greater than the mean change value
                if n > mean_change_value:
                    significant_change += n-mean_change_value
            accumulated_change[i][j] = significant_change
            ### ======================================================================= ###

    return accumulated_change

# Checks whether the piece is white
def is_white(piece):
    return not is_empty(piece) and piece.isupper()

# Checks whether the piece is black
def is_black(piece):
    return not is_empty(piece) and piece.islower()

# Checks whether the piece is empty
def is_empty(piece):
    return piece == '-'

# Find the move that the user played
def find_move(state, positions, white):

    # Regular move case
    if len(positions) == 2:
        # Get matrix coordinates and convert to chess coordinates
        [r1, c1], [r2, c2] = positions
        p1, p2 = convert_to_chess_notation([r1, c1], white), convert_to_chess_notation([r2, c2], white)

        if is_empty(state[r1][c1]):
            # Position 1 empty, Position 2 should have piece:
            # Move: Position 2 moves to Position 1
            move = f'{p2}{p1}'

        elif is_black(state[r1][c1]):
            if white:
                # Position 1 black, Position 2 should be white or empty
                # Move: Position 1 moves to Position 2
                move = f'{p1}{p2}'
            else:
                # Position 1 black, Position 2 should be white
                # Move: Position 2 moves to Position 1
                move = f'{p2}{p1}'

        elif is_white(state[r1][c1]):
            if not white:
                # Position 1 white, Position should be 2 black or empty
                # Move: Position 1 moves to Position 2
                move = f'{p1}{p2}'
            else:
                # Position 1 white, Position 2 should be black
                # Move: Position 2 moves to Position 1
                move = f'{p2}{p1}'

    # Castling move case
    elif len(positions) == 4:
        # Initial and final positions of King and Rook
        ki, kf = 'a0', 'a0'
        if white:
            for n in positions:
                p = convert_to_chess_notation(n, white)
                if state[n[0]][n[1]] == 'k':
                    ki = p
                if state[n[0]][n[1]-2] == 'k' or state[n[0]][n[1]+2] == 'k':
                    kf = p
        else:
            for n in positions:
                p = convert_to_chess_notation(n, white)
                if state[n[0]][n[1]] == 'K':
                    ki = p
                if state[n[0]][n[1]-2] == 'K' or state[n[0]][n[1]+2] == 'K':
                    kf = p

        move = f'{ki}{kf}'

    # Error in detecting move
    else:
        move = 'a0a0'
        
    return move

# Takes in images and returns the move the user played
def get_move(IMG_1_PATH, IMG_2_PATH, side, state):

    if side == "white":
        WHITE = True
    else:
        WHITE = False

    # Desired image dimensions
    DIMS = (512, 512)
    # Threshold percentage value
    THRESHOLD_VALUE = 0.6
    # Chessboard location on image
    UPPER_LEFT_CORNER = (675, 0)
    LOWER_RIGHT_CORNER = (1756, 1079)

    # Get the matrix containing the total change for each square
    change = get_change(IMG_1_PATH, IMG_2_PATH, DIMS, [UPPER_LEFT_CORNER, LOWER_RIGHT_CORNER], display = False)

    # Get most likely positions where a changed occured
    positions = get_locations_of_max_values(change, THRESHOLD_VALUE)

    # Find the move that happened and update the chessboard
    user_move = find_move(state, positions, WHITE)

    return user_move
