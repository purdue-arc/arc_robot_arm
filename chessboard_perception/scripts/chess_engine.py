import chess
import chess.engine


class ChessEngine:
    # Initialize attributes
    def __init__(self, state_fen, side):
        self.board = chess.Board(state_fen)
        self.engine = chess.engine.SimpleEngine.popen_uci("Stockfish\stockfish_14.1_linux_x64")
        if side == 'white':
            self.side = chess.WHITE
        else:
            self.side = chess.BLACK

    # Checks if it's the robot's turn to play
    def is_robot_turn(self):
        return self.board.turn == self.side

    # Checks if the given move is legal
    def check_move(self, move):
        return chess.Move.from_uci(move) in self.board.legal_moves

    # Makes the user's move on the board
    def make_move(self, move):
        # Check if move is a promotion
        if chess.Move.from_uci(move+"q") in self.board.legal_moves:
            while True:
                promo_piece = input("Enter Promoted Piece ('q', 'r', 'n', 'b'): ")
                if promo_piece in ['q', 'r', 'n', 'b']:
                    break
                else:
                    print("Try Again.")
            move += promo_piece
        if self.check_move(move):
            self.board.push_uci(move)
            return True
        return False

    # Plays the AI's move onto the board and returns it for the robot to play
    def AI_move(self):
        result = self.engine.play(self.board, chess.engine.Limit(time=0.1))
        result_lan = (self.board.lan(result.move))
        self.board.push(result.move)
        return result_lan

    # Checks whether the game board is playable
    def is_playable(self):
        return not (self.board.is_checkmate() or self.board.is_stalemate() or self.board.is_insufficient_material())

    # Returns the robot's side
    def get_side(self):
        if self.side == chess.WHITE:
            return "white"
        return "black"

    # Converts FEN state into a matrix
    def get_state_matrix(self):
        state_fen = self.get_state_fen().split(' ')[0]
        if self.side == chess.WHITE:
            state_fen_split = state_fen.split('/')
        else:
            reversed_state_fen = ""
            for n in state_fen:
                reversed_state_fen = n + reversed_state_fen
            state_fen_split = reversed_state_fen.split('/')
        state_matrix = list()
        for row in state_fen_split:
            matRow = list()
            for square in list(row):
                if square.isnumeric():
                    for i in range(int(square)):
                        matRow.append("-")
                else:
                    matRow.append(square)
            state_matrix.append(matRow)
        return state_matrix

    # Returns board's state in FEN
    def get_state_fen(self):
        return self.board.fen()

    # Converts the move into coordinates
    def convert_to_coordinates(self, move):
        if '=' in move and 'x' in move:
            # Capture and Promotion
            m = move.split("x")
            p1 = m[1][0:2]
            p2 = "*"
            p3 = m[0][-2:]
            p4 = "*"
            p5 = "$"+m[1][3]
            p6 = m[1][0:2]
            positions = [p1, p2, p3, p4, p5, p6]
        elif '=' in move:
            # Promotion
            m = move.split("-")
            p1 = m[0][-2:]
            p2 = "*"
            p3 = "$"+m[1][3]
            p4 = m[1][0:2]
            positions = [p1, p2, p3, p4]
        elif 'x' in move:
            # Capture
            m = move.split("x")
            p1 = m[1][0:2]
            p2 = "*"
            p3 = m[0][-2:]
            p4 = m[1][0:2]
            positions = [p1, p2, p3, p4]
        elif 'O-O-O' in move:
            # Queenside Castle
            if self.side == chess.WHITE:
                p1 = "e1"
                p2 = "c1"
                p3 = "a1"
                p4 = "d1"
            else:
                p1 = "d1"
                p2 = "f1"
                p3 = "h1"
                p4 = "e1"
            positions = [p1, p2, p3, p4]
        elif 'O-O' in move:
            # Kingside Castle
            if self.side == chess.WHITE:
                p1 = "e1"
                p2 = "g1"
                p3 = "h1"
                p4 = "f1"
            else:
                p1 = "d1"
                p2 = "b1"
                p3 = "a1"
                p4 = "c1"
            positions = [p1, p2, p3, p4]
        else:
            # Regular Move
            m = move.split("-")
            p1 = m[0][-2:]
            p2 = m[1][0:2]
            positions = [p1, p2]

        key = {'a': 1, 'b': 2, 'c': 3, 'd': 4, 'e': 5, 'f': 6, 'g': 7, 'h': 8}
        promotion_key = {'Q': -2, 'R': -3, 'B': -4, 'N': -5}
        for i in range(len(positions)):
            if positions[i] == "*":
                positions[i] = [-1]
            elif "$" in positions[i]:
                positions[i] = [promotion_key[positions[i][1]]]
            else:
                if self.side == chess.WHITE:
                    x, y = (key[positions[i][0]]-0.5), (int(positions[i][1])-0.5)
                else:
                    x, y = (8.5-key[positions[i][0]]), (8.5-int(positions[i][1]))
                positions[i] = [x, y]
        return positions
