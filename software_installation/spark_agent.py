        self.send_command('(scene rsg/agent/naov4/nao.rsg)')
        self.sense()  # only need to get msg from simspark
        init_cmd = ('(init (unum ' + str(player_id) + ')(teamname ' + teamname + '))')
        self.send_command(init_cmd)
        self.thread = None

        while player_id == 0:
            self.sense()
            self.send_command('')
            player_id = self.perception.game_state.unum
        self.player_id = player_id

    def act(self, action):
        commands = action.to_commands()
        self.send_command(commands)

    def send_command(self, commands):
        if self.sync_mode:
            commands += '(syn)'
        self.socket.sendall(struct.pack(b"!I", len(commands)) + bytes(commands, encoding='utf8'))

    def connect(self, simspark_ip, simspark_port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((simspark_ip, simspark_port))

    def sense(self):
        length = b''
        while(len(length) < 4):
            length += self.socket.recv(4 - len(length))
        length = struct.unpack("!I", length)[0]
        msg = b''
        while len(msg) < length:
            msg += self.socket.recv(length - len(msg))

        sexp = str2sexpr(msg.decode())
        self.perception.update(sexp)
        return self.perception

    def think(self, perception):
        action = Action()
        return action

    def sense_think_act(self):
        perception = self.sense()
        action = self.think(perception)
        self.act(action)

    def run(self):
        while True:
            self.sense_think_act()

    def start(self):
        if self.thread is None:
            self.thread = Thread(target=self.run)
            self.thread.daemon = True
            self.thread.start()


if '__main__' == __name__:
    agent = SparkAgent()
    agent.run()
