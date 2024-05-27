#!/usr/bin/env python3

import rospy
from gomoku_vision.msg import Position
from __future__ import print_function
import pickle
from game import Board, Game
from mcts_alphaZero import MCTSPlayer
from policy_value_net_numpy import PolicyValueNetNumpy

n = 5
width, height = 8, 8
"""
In the open source code we called, board structure is quite different.
TODO: modify board structure with subscribe position.
"""

class GomokuAINode:
    def __init__(self):
        self.board = Board(width=width, height=height, n_in_row=n)
        self.game = Game(self.board)
        self.move_pub = rospy.Publisher('next_move', Position, queue_size=10)
        self.position_sub = rospy.Subscriber('piece_position', Position, self.piece_callback)
        self.model_file = 'best_policy_8_8_5.model'
        
        # Load the policy-value network
        policy_param = pickle.load(open(self.model_file, 'rb'), encoding='bytes')
        self.best_policy = PolicyValueNetNumpy(width, height, policy_param)
        self.mcts_player = MCTSPlayer(self.best_policy.policy_value_fn, c_puct=5, n_playout=400)
    
    def run(self):
        rospy.spin()

    def piece_callback(self, data):
        # Convert received position
        received_x = 7 - data.x
        received_y = data.y
        
        # Update the board with the human move
        human_move = self.board.location_to_move((received_x, received_y))
        self.board.do_move(human_move)
        
        # Get the AI move
        ai_move = self.mcts_player.get_action(self.board)
        
        # Convert AI move back to (x, y) position
        ai_move_x, ai_move_y = self.board.move_to_location(ai_move)
        
        # Convert AI move to publishable position
        publish_x = 7 - ai_move_x
        publish_y = ai_move_y
        
        # Publish the AI move
        ai_move_position = Position()
        ai_move_position.x = publish_x
        ai_move_position.y = publish_y
        self.move_pub.publish(ai_move_position)
        
        # Update the board with the AI move
        self.board.do_move(ai_move)

if __name__ == '__main__':
    rospy.init_node('gomoku_ai_node')
    node = GomokuAINode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
