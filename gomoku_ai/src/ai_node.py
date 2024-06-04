#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(__file__))

import rospy
import pickle
from std_msgs.msg import Int8, UInt8MultiArray
from game import Board, Game
from mcts_alphaZero import MCTSPlayer
from policy_value_net_numpy import PolicyValueNetNumpy

n = 5
width, height = 8, 8

class Human(object):
    """
    human player
    """

    def __init__(self):
        self.player = None

    def set_player_ind(self, p):
        self.player = p

    def get_action(self, board):
        try:
            location = input("Your move: ")
            if isinstance(location, str):  # for python3
                location = [int(n, 10) for n in location.split(",")]
            move = board.location_to_move(location)
        except Exception as e:
            move = -1
        if move == -1 or move not in board.availables:
            print("invalid move")
            move = self.get_action(board)
        return move

    def __str__(self):
        return "Human {}".format(self.player)

class GomokuAINode:
    def __init__(self):
        self.model_file = os.path.join(os.path.dirname(__file__), 'best_policy_8_8_5.model')
        with open(self.model_file, 'rb') as f:
            policy_param = pickle.load(f, encoding='bytes')
        self.board = Board(width=width, height=height, n_in_row=n)
        self.game = Game(self.board)
        self.move_pub = rospy.Publisher('/next_move', UInt8MultiArray, queue_size=10)
        self.position_sub = rospy.Subscriber('/piece_position', UInt8MultiArray, self.piece_callback)
        
        # Load the policy-value network
        policy_param = pickle.load(open(self.model_file, 'rb'), encoding='bytes')
        self.best_policy = PolicyValueNetNumpy(width, height, policy_param)
        self.mcts_player = MCTSPlayer(self.best_policy.policy_value_fn, c_puct=5, n_playout=400)
        self.human = Human()
        self.player1, self.player2 = self.human, self.mcts_player
        self.game.start_play(self.player1, self.player2, start_player=0, is_shown=0)
        rospy.loginfo("Board Initialized")
    
    def run(self):
        rospy.spin()

    def piece_callback(self, receive):
        # Convert received position
        x = receive.data[0] - 1
        y = receive.data[1] - 1

        received_x = 7 - x
        received_y = y
        rospy.loginfo("received message")
        self.game.graphic(self.board, self.player1.player, self.player2.player)
        
        # Update the board with the human move
        human_move = self.board.location_to_move((received_x, received_y))
        self.board.do_move(human_move)
        
        # Get the AI move
        ai_move = self.mcts_player.get_action(self.board)

        rospy.loginfo("Get AI move")
        
        # Convert AI move back to (x, y) position
        ai_move_x, ai_move_y = self.board.move_to_location(ai_move)
        rospy.loginfo("Convert message to x, y")
        
        # Convert AI move to publishable position
        publish_x = 7 - ai_move_x + 1
        publish_y = ai_move_y + 1
        
        # Publish the AI move
        ai_move_position = UInt8MultiArray()
        ai_move_position.data = [publish_x, publish_y]
        self.move_pub.publish(ai_move_position)
        rospy.loginfo("Published")
        rospy.loginfo(publish_x)
        rospy.loginfo(publish_y)
        
        # Update the board with the AI move
        self.board.do_move(ai_move)
        rospy.loginfo("Board updated")
        self.game.graphic(self.board, self.player1.player, self.player2.player)

if __name__ == '__main__':
    rospy.init_node('gomoku_ai_node')
    rospy.loginfo("node initialized")
    node = GomokuAINode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
