import socket, cPickle, zlib, threading, Queue, sys, os 
#not default:
import numpy, pyglet
#self authored:
import projector

#Server to drive projector mounted on robot    

#class to namespace operating variables
class opNS:
    def __init__(self):
        self.mdverts = []; self.mdfaces = []; self.period = .1; self.animdex = 0; self.fs = []; self.vs = []; self.gupdate = False; self.onedim = True
        self.VC0 = 4; self.F0 = [0,1,2,0,2,3]; self.VER0 = (100, 100, 150, 100, 150, 150, 100, 150); self.colors = ((0,255,0)*self.VC0) 
        self.vertex_list = pyglet.graphics.vertex_list_indexed(self.VC0, self.F0, ('v2f', self.VER0), ('c3B', self.colors))
        self.label = pyglet.text.Label('', font_name='Arial', font_size=36,x=window.width//2, y=window.height//2, anchor_x='center', anchor_y='center')


def server():
    HOST = ''             
    PORT = 50007          

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    while 1:
        conn, addr = s.accept()
        while 1:
            data = getDataFromSocket(conn)
            if not data: break
            try: dobj = cPickle.loads(zlib.decompress(data))
            except: break
            q.put(dobj)
        conn.close()


def pidFile(create=True, pidfile='.glpid'):
    if create:
        if os.path.isfile(pidfile):
            sys.exit()
        else:
            pid = os.getpid()
            cPickle.dump(pid, open(pidfile, 'w'))
    else:
        os.remove(pidfile)


#allows small buffer size to be used even for large pickled messages
#switched to length header from stupid timeout, but you do need to make sure
#HEADERLENGTH is the same in sendPickles(obj) as well
def getDataFromSocket(sck):
    HEADERLENGTH = 16; data = ""    
    data = sck.recv(2048)
    RECLEN = data[0:HEADERLENGTH]
    if (len(RECLEN) == 0): return None
    else: 
        try: RECLEN = int(RECLEN)
        except: return None

    data = data[HEADERLENGTH:len(data)]
    
    while (len(data) < RECLEN):
        line = ""
        try: line = sck.recv(2048)
        except socket.timeout: break
        if (line == ""): break
        data += line
    
    if (len(data) == RECLEN): return data
    else: return None 


def object2vl(obj):
    if projector.GLtmCheck(obj):
        obj.makeInt()
        v.mdverts = obj.verts
        v.mdfaces = obj.faces
        v.colors = obj.colors
        v.period = obj.period
        v.fs = numpy.shape(v.mdfaces); v.vs = numpy.shape(v.mdverts); cs = numpy.shape(v.colors)
        v.label = pyglet.text.Label(obj.label[0], font_name='Arial', font_size=36,x=obj.label[1], y=obj.label[2], anchor_x='center', anchor_y='center')
        if (len(v.vs) == 1): 
            v.vcount = v.vs[0]/2; v.onedim = True
            v.vertex_list = pyglet.graphics.vertex_list_indexed(v.vcount, v.mdfaces, ('v2f', v.mdverts), ('c3B', v.colors))
        else: 
            v.vcount = v.vs[1]/2; v.onedim = False
            v.vertex_list = pyglet.graphics.vertex_list_indexed(v.vcount, v.mdfaces, ('v2f', v.mdverts[0]), ('c3B', v.colors))
        v.animdex = 0; v.gupdate = True
    else:
        print "GLTM object invalid"


# fuction is scheduled into event loop
def update(dt):
    #case where message is recieved
    if (not q.empty()):    
        obj = q.get()
        object2vl(obj)
    if v.gupdate:
        if v.onedim:
            v.vertex_list.vertices = v.mdverts
        else: 
            if (v.animdex < (v.vs[0]-1)): v.animdex += 1
            else: v.animdex = 0
            v.vertex_list.vertices = v.mdverts[v.animdex]






# get system information, so we can choose appropriate screen
platform = pyglet.window.get_platform()
display = platform.get_default_display()
screens = display.get_screens()


if (sys.platform == 'linux2'):
    config = pyglet.gl.Config()
else:
    #enables multisample antialiasing (only works on windows for some reason)
    config = pyglet.gl.Config(sample_buffers=1, samples=4)

print 'screens detected:', len(screens)
local = '-l' in sys.argv
if ((len(screens) == 1) | local):
    window = pyglet.window.Window(config=config, fullscreen=False, width=800, height=600, screen=screens[0])
else:
    #For when projector is connected as second monitor:
    print screens
    window = pyglet.window.Window(config=config, fullscreen=True, screen=screens[-1])


@window.event
def on_draw():
    window.clear()
    v.vertex_list.draw(pyglet.gl.GL_TRIANGLES)
    v.label.draw()



print window.get_size()
try:
    #pidFile(True)
    # start the queue for data exchange
    q = Queue.Queue()
    q.put(projector.ccbGLTM(200))

    #start the server
    t = threading.Thread(target=server)
    t.daemon = True; t.start()

    #create object to hold variables
    v = opNS()

    #schedule graphics update
    pyglet.clock.schedule_interval(update, v.period)

    #start event loop
    pyglet.app.run()

finally:
    pass #pidFile(False)
