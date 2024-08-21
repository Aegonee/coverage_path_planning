from PyQt5.Qt import *
import sys

class Window(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('QAbstractButton-点击加一')
        self.resize(500, 400)
        self.setup_ui()
    
    def setup_ui(self):
        lable = QLabel(self)
        lable.setText('1')
        
        btn = QPushButton(self)
        btn.setText('加一')
        btn.move(0, 20)
        def incr(evt):
            text = int(lable.text()) + 1
            lable.setText(str(text))
        btn.clicked.connect(incr)
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
