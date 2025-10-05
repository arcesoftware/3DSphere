package main

import (
	"fmt"
	"log"
	"math"
	"math/rand"
	"runtime"
	"time"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/go-gl/mathgl/mgl32"
)

const (
	winWidth   = 1080
	winHeight  = 1400
	nParticles = 350000
	pointSize  = 4.0
)

type Particle struct {
	Pos mgl32.Vec3
	Vel mgl32.Vec3
	Col mgl32.Vec3
}

var (
	particles []Particle

	prog   uint32
	vao    uint32
	vboPos uint32
	vboCol uint32

	azimuth, elevation float64 = 0.6, 0.2
	distance           float64 = 900
	lastX, lastY       float64
	dragging           bool
)

func init() {
	runtime.LockOSThread()
}

func main() {
	rand.Seed(time.Now().UnixNano())
	if err := glfw.Init(); err != nil {
		log.Fatalln("failed to init glfw:", err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(winWidth, winHeight, "3D Particles", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()
	if err := gl.Init(); err != nil {
		panic(err)
	}

	fmt.Println("OpenGL version", gl.GoStr(gl.GetString(gl.VERSION)))
	gl.Enable(gl.DEPTH_TEST)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.PointSize(pointSize)

	prog, err = newProgram(vertexShader, fragmentShader)
	if err != nil {
		panic(err)
	}

	particles = make([]Particle, nParticles)
	for i := 0; i < nParticles; i++ {
		particles[i].Pos = mgl32.Vec3{
			float32((rand.Float64()*2 - 1) * 300),
			float32((rand.Float64()*2 - 1) * 300),
			float32((rand.Float64()*2 - 1) * 300),
		}
		particles[i].Vel = mgl32.Vec3{0, 0, 0}
		r := float32(rand.Float64()*0.6 + 0.4)
		g := float32(rand.Float64()*0.6 + 0.1)
		b := float32(rand.Float64() * 0.3)
		particles[i].Col = mgl32.Vec3{r, g, b}
	}

	setupBuffers()
	window.SetCursorPosCallback(mouseMove)
	window.SetMouseButtonCallback(mouseButton)
	window.SetScrollCallback(scrollCallback)

	prev := time.Now()
	for !window.ShouldClose() {
		now := time.Now()
		dt := now.Sub(prev).Seconds()
		prev = now

		updateParticles(dt)
		render()
		window.SwapBuffers()
		glfw.PollEvents()
	}
}

// --------- Particle Dynamics ---------
func updateParticles(dt float64) {
	const sphereRadius = 150.0
	const springStrength = 50.0
	const damping = 0.05
	dt32 := float32(dt)

	for i := range particles {
		p := &particles[i]
		pos := p.Pos
		dist := float64(pos.Len())
		if dist > 0 {
			dir := pos.Normalize()
			force := springStrength * (dist - sphereRadius)
			acc := dir.Mul(-float32(force))
			p.Vel = p.Vel.Add(acc.Mul(dt32))
		}
		p.Vel = p.Vel.Mul(1.0 - damping)
		p.Pos = p.Pos.Add(p.Vel.Mul(dt32))
	}
}

// --------- Rendering ---------
func setupBuffers() {
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	// Positions
	gl.GenBuffers(1, &vboPos)
	gl.BindBuffer(gl.ARRAY_BUFFER, vboPos)
	gl.BufferData(gl.ARRAY_BUFFER, nParticles*3*4, nil, gl.DYNAMIC_DRAW)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))

	// Colors
	gl.GenBuffers(1, &vboCol)
	gl.BindBuffer(gl.ARRAY_BUFFER, vboCol)
	gl.BufferData(gl.ARRAY_BUFFER, nParticles*3*4, nil, gl.DYNAMIC_DRAW)
	gl.EnableVertexAttribArray(1)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))

	gl.BindVertexArray(0)
}

func render() {
	gl.ClearColor(0, 0, 0, 1)
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.UseProgram(prog)

	proj := mgl32.Perspective(mgl32.DegToRad(45), float32(winWidth)/winHeight, 0.1, 5000)
	view := cameraMatrix()
	vp := proj.Mul4(view)
	loc := gl.GetUniformLocation(prog, gl.Str("uVP\x00"))
	gl.UniformMatrix4fv(loc, 1, false, &vp[0])

	posData := make([]float32, 0, nParticles*3)
	colData := make([]float32, 0, nParticles*3)
	for _, p := range particles {
		posData = append(posData, p.Pos.X(), p.Pos.Y(), p.Pos.Z())
		colData = append(colData, p.Col.X(), p.Col.Y(), p.Col.Z())
	}

	gl.BindBuffer(gl.ARRAY_BUFFER, vboPos)
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, len(posData)*4, gl.Ptr(posData))
	gl.BindBuffer(gl.ARRAY_BUFFER, vboCol)
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, len(colData)*4, gl.Ptr(colData))

	gl.BindVertexArray(vao)
	gl.DrawArrays(gl.POINTS, 0, int32(nParticles))
	gl.BindVertexArray(0)
}

// --------- Camera ---------
func cameraMatrix() mgl32.Mat4 {
	x := float32(distance * math.Cos(azimuth) * math.Cos(elevation))
	y := float32(distance * math.Sin(elevation))
	z := float32(distance * math.Sin(azimuth) * math.Cos(elevation))
	return mgl32.LookAtV(mgl32.Vec3{x, y, z}, mgl32.Vec3{0, 0, 0}, mgl32.Vec3{0, 1, 0})
}

func mouseMove(w *glfw.Window, xpos, ypos float64) {
	if dragging {
		azimuth -= (xpos - lastX) * 0.005
		elevation -= (ypos - lastY) * 0.005
		if elevation > 1.4 {
			elevation = 1.4
		} else if elevation < -1.4 {
			elevation = -1.4
		}
	}
	lastX = xpos
	lastY = ypos
}

func mouseButton(w *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
	if button == glfw.MouseButton1 {
		dragging = action == glfw.Press
	}
}

func scrollCallback(w *glfw.Window, xoff, yoff float64) {
	distance -= yoff * 30
	if distance < 200 {
		distance = 200
	}
	if distance > 3000 {
		distance = 3000
	}
}

// --------- Shader helpers ---------
func compileShader(src string, shaderType uint32) (uint32, error) {
	shader := gl.CreateShader(shaderType)
	csources, free := gl.Strs(src)
	gl.ShaderSource(shader, 1, csources, nil)
	free()
	gl.CompileShader(shader)
	var status int32
	gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLength)
		logBuf := make([]byte, logLength+1)
		gl.GetShaderInfoLog(shader, logLength, nil, &logBuf[0])
		return 0, fmt.Errorf("failed to compile shader: %s", string(logBuf))
	}
	return shader, nil
}

func newProgram(vertSrc, fragSrc string) (uint32, error) {
	vert, err := compileShader(vertSrc, gl.VERTEX_SHADER)
	if err != nil {
		return 0, err
	}
	frag, err := compileShader(fragSrc, gl.FRAGMENT_SHADER)
	if err != nil {
		return 0, err
	}
	prog := gl.CreateProgram()
	gl.AttachShader(prog, vert)
	gl.AttachShader(prog, frag)
	gl.LinkProgram(prog)
	var status int32
	gl.GetProgramiv(prog, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var logLength int32
		gl.GetProgramiv(prog, gl.INFO_LOG_LENGTH, &logLength)
		logBuf := make([]byte, logLength+1)
		gl.GetProgramInfoLog(prog, logLength, nil, &logBuf[0])
		return 0, fmt.Errorf("failed to link program: %s", string(logBuf))
	}
	gl.DeleteShader(vert)
	gl.DeleteShader(frag)
	return prog, nil
}

// --------- Shaders ---------
var vertexShader = `
#version 410 core
layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inCol;
uniform mat4 uVP;
out vec3 vColor;
void main() {
    gl_Position = uVP * vec4(inPos, 1.0);
    vColor = inCol;
}
` + "\x00"

var fragmentShader = `
#version 410 core
in vec3 vColor;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, 1.0);
}
` + "\x00"
