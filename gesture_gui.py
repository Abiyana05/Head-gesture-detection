import sys
import serial
from PyQt5 import QtWidgets, QtCore
from playsound import playsound
from threading import Thread


class GestureGUI(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Create a QTextEdit to display gesture data
        self.text_edit = QtWidgets.QTextEdit(self)
        self.text_edit.setAlignment(QtCore.Qt.AlignTop)
        self.text_edit.setStyleSheet("font-size: 24px; color: white; background-color: black;")
        self.text_edit.setReadOnly(True)  # Make it read-only

        # Create a button to send the character to enable DMP
        self.enable_button = QtWidgets.QPushButton("Enable DMP", self)
        self.enable_button.setStyleSheet("font-size: 24px;")
        self.enable_button.clicked.connect(self.sendEnableDMP)

        # Set up layout
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.text_edit)  # Add text_edit to layout
        layout.addWidget(self.enable_button, alignment=QtCore.Qt.AlignBottom)  # Add button at the bottom

        # Set up window properties for fullscreen
        self.setWindowTitle('Gesture Detector')
        self.setStyleSheet("background-color: black;")
        self.showFullScreen()

        # Initialize serial communication
        try:
            self.serial_port = serial.Serial('COM3', 115200, timeout=1)
        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(self, "Serial Port Error", f"Error opening COM port: {e}")
            sys.exit()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.readSerial)
        self.timer.start(100)

        # Timer to reset display
        self.display_timer = QtCore.QTimer(self)
        self.display_timer.setSingleShot(True)
        self.display_timer.timeout.connect(self.resetToNormal)

        # Define gestures with their respective audio
        self.gesture_media = {
            "Left-Right-Left-Right-Neutral sequence completed!": {
                "audio": "neutral left right left right neutral.wav"  # Path to audio for this gesture
            },
            "Left movement sequence completed!": {
                "audio": "neutral left neutral .wav"  # Path to audio for this gesture
            },
            "Right movement sequence completed!": {
                "audio": "neutral right neutral .wav"  # Path to audio for this gesture
            },
            "Neutral-Up-Neutral gesture completed!": {
                "audio": "neutral up neutral.wav"  # Path to audio for this gesture
            },
            "Neutral-Down-Neutral gesture completed!": {
                "audio": "neutral down neutral.wav"  # Path to audio for this gesture
            },
            "Down-Up-Down-Up-Neutral sequence completed!": {
                "audio": "neutral down up down up neutral.wav"  # Path to audio for this gesture
            }
        }

    def sendEnableDMP(self):
        """Send a character to enable DMP."""
        if self.serial_port.is_open:
            self.serial_port.write(b'e')  # Replace 'e' with the appropriate character for enabling DMP
            self.text_edit.append("DMP Enabled: Character sent!")  # Update QTextEdit

    def readSerial(self):
        """Read gesture data from the serial port."""
        if self.serial_port.in_waiting > 0:
            try:
                data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                self.text_edit.append(data)  # Append the new data to the QTextEdit
            except Exception as e:
                print(f"Error reading serial data: {e}")
                return

            # Check if the data matches any predefined gestures
            if data in self.gesture_media:
                # Play sound
                self.playSoundThread(self.gesture_media[data]["audio"])

                # Show black screen with gesture-specific text
                self.showBlackScreen(data)

    def showBlackScreen(self, gesture_text):
        """Display a black page with large text for 4 seconds."""
        self.text_edit.hide()  # Hide the text edit
        self.enable_button.hide()  # Hide the button

        # Define custom text for each gesture
        gesture_display_texts = {
            "Left-Right-Left-Right-Neutral sequence completed!": "Left-Right-Left-Right-Neutral sequence completed",
            "Left movement sequence completed!": "Left movement sequence completed!",
            "Right movement sequence completed!": "Right movement sequence completed!",
            "Neutral-Up-Neutral gesture completed!": "Neutral-Up-Neutral gesture completed!",
            "Neutral-Down-Neutral gesture completed!": "Neutral-Down-Neutral gesture completed",
            "Down-Up-Down-Up-Neutral sequence completed!": "Down-Up-Down-Up-Neutral sequence completed!."
        }

        # Get the text to display
        display_text = gesture_display_texts.get(gesture_text, "Gesture detected!")

        self.black_screen_label = QtWidgets.QLabel(self)
        self.black_screen_label.setText(display_text)
        self.black_screen_label.setStyleSheet("font-size: 64px; color: white; background-color: black;")
        self.black_screen_label.setAlignment(QtCore.Qt.AlignCenter)
        self.black_screen_label.setGeometry(0, 0, self.width(), self.height())
        self.black_screen_label.show()

        # Start timer to reset the display
        self.display_timer.start(6000)

    def resetToNormal(self):
        """Reset the display to its normal state."""
        if hasattr(self, 'black_screen_label'):
            self.black_screen_label.hide()  # Hide black screen label
            self.black_screen_label.deleteLater()

        self.text_edit.show()  # Show the text edit
        self.enable_button.show()  # Show the button

    def playSoundThread(self, sound_file):
        """Play sound in a separate thread."""
        Thread(target=playsound, args=(sound_file,)).start()

    def resizeEvent(self, event):
        """Resize components when the window is resized."""
        self.text_edit.setGeometry(0, 0, self.width(), self.height() - 100)  # Adjust size on resize
        super().resizeEvent(event)

    def closeEvent(self, event):
        """Handle application close event."""
        if self.serial_port.is_open:
            self.serial_port.close()
        event.accept()


# Main application loop
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = GestureGUI()
    sys.exit(app.exec_())
