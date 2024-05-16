#!usr/bin/env python3

from enum import Enum

N = 9
class BoardState(Enum):
    EMPTY = 0
    BLACK = 1
    WHITE = 2
    
class GomokuBoard:
    def __init__(self):
        self.board = [[BoardState.EMPTY for _ in range(N)] for _ in range(N)]
        self.current_player = BoardState.BLACK
        self.winner = BoardState.EMPTY
        self.game_over = False

    def reset(self):
        self.board = [[BoardState.EMPTY for _ in range(N)] for _ in range(N)]
        self.current_player = BoardState.BLACK
        self.winner = BoardState.EMPTY
        self.game_over = False

    def play(self, x, y):
        if self.game_over:
            return
        if self.board[y][x] == BoardState.EMPTY:
            self.board[y][x] = self.current_player
            if self.check_win(x, y):
                self.winner = self.current_player
                self.game_over = True
            self.current_player = BoardState.BLACK if self.current_player == BoardState.WHITE else BoardState.WHITE

    def check_win(self, x, y):
        if self.check_direction(x, y, 1, 0) + self.check_direction(x, y, -1, 0) >= 4:
            return True
        if self.check_direction(x, y, 0, 1) + self.check_direction(x, y, 0, -1) >= 4:
            return True
        if self.check_direction(x, y, 1, 1) + self.check_direction(x, y, -1, -1) >= 4:
            return True
        if self.check_direction(x, y, 1, -1) + self.check_direction(x, y, -1, 1) >= 4:
            return True
        return False

    def check_direction(self, x, y, dx, dy):
        count = 0
        while x >= 0 and x < N and y >= 0 and y < N and self.board[y][x] == self.current_player:
            count += 1
            x += dx
            y += dy
        return count

    def get_board(self):
        return self.board

    def get_current_player(self):
        return self.current_player

    def get_winner(self):
        return self.winner

    def is_game_over(self):
        return self.game_over

    def get_state(self):
        return self.board, self.current_player, self.winner, self.game_over