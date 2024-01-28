import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNote
from builtin_interfaces.msg import Duration

class AudioNoteSequenceClient(Node):

    def __init__(self):
        super().__init__('audio_note_sequence_client')
        self.client = ActionClient(self, AudioNoteSequence, '/audio_note_sequence')

    def send_goal(self, note_sequence):
        goal_msg = AudioNoteSequence.Goal()
        goal_msg.iterations = 1
        goal_msg.note_sequence.append = False
        goal_msg.note_sequence.notes = note_sequence

        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)

    def melody(self, notes_str, bpm=120):
        tempo = 60000000000/bpm
        s, ns = divmod(tempo,1000000000)
        notes = notes_str.split()
        note_sequence = []
        for note in notes:
            f = self.note_to_frequency(note)
            note_sequence.append(AudioNote(frequency=f, max_runtime=Duration(sec=int(s), nanosec=int(ns))))
        return note_sequence

    def note_to_frequency(self, note):
        note_frequencies = {
            "C3": 130.81, "C3#": 138.59, "D3b": 138.59, "D3": 146.83, "D3#": 155.56, "E3b": 155.56,
            "E3": 164.81, "F3": 174.61, "F3#": 185.00, "G3b": 185.00, "G3": 196.00, "G3#": 207.65,
            "A3b": 207.65, "A3": 220.00, "A3#": 233.08, "B3b": 233.08, "B3": 246.94,
            "C4": 261.63, "C4#": 277.18, "D4b": 277.18, "D4": 293.66, "D4#": 311.13, "E4b": 311.13,
            "E4": 329.63, "F4": 349.23, "F4#": 369.99, "G4b": 369.99, "G4": 392.00, "G4#": 415.30,
            "A4b": 415.30, "A4": 440.00, "A4#": 466.16, "B4b": 466.16, "B4": 493.88,
            "C5": 523.25, "C5#": 554.37, "D5b": 554.37, "D5": 587.33, "D5#": 622.25, "E5b": 622.25,
            "E5": 659.25, "F5": 698.46, "F5#": 739.99, "G5b": 739.99, "G5": 783.99, "G5#": 830.61,
            "A5b": 830.61, "A5": 880.00, "A5#": 932.33, "B5b": 932.33, "B5": 987.77,
            "C6": 1046.50, "C6#": 1108.73, "D6b": 1108.73, "D6": 1174.66, "D6#": 1244.51, "E6b": 1244.51,
            "E6": 1318.51, "F6": 1396.91, "F6#": 1479.98, "G6b": 1479.98, "G6": 1567.98, "G6#": 1661.22,
            "A6b": 1661.22, "A6": 1760.00, "A6#": 1864.66,"B6b": 1864.66, "B6": 1975.53, "C7": 2093.00
        }
        return int(round(note_frequencies.get(note, 0),0))  # Default to 0 if note not found
    
def main(args=None):
    rclpy.init(args=args)
    action_client = AudioNoteSequenceClient()
    note_sequence = action_client.melody("C3 D3 D3# D3 D3# F4 F4# F4 A4# G4")
    print(note_sequence)
    action_client.send_goal(note_sequence)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
