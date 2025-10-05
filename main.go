package main

// 3D Particle Simulation using Go + OpenGL (go-gl)
// - Renders particles as GL_POINTS with depth and perspective
// - Camera: orbit (mouse drag), zoom (scroll)
// - Dynamic buffer updates each frame (positions & colors)
// Requirements:
//   go get github.com/go-gl/gl/v4.1-core/gl
//   go get github.com/go-gl/glfw/v3.3/glfw
//   go get github.com/go-gl/mathgl/mgl32
// Also ensure GLFW native libraries are installed on your system.

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
	winWidth   = 1200
	winHeight  = 800
	nParticles = 3500
	pointSize  = 8.0
)

// Particle holds 3D position, velocity and RGB color
type Particle struct {
	Pos mgl32.Vec3
	Vel mgl32.Vec3
	Col mgl32.Vec3
}

var (
	particles []Particle
	prog      uint32
	vboPos    uint32
	vboCol    uint32
	vao       uint32

	// camera params
	azimuth   float64 = 0.6
	elevation float64 = 0.2
	distance  float64 = 900.0
	lastX     float64
	lastY     float64
	dragging  bool
)

func init() {
	// GLFW must run on main OS thread
	runtime.LockOSThread()
}

func main() {
	rand.Seed(time.Now().UnixNano())

	if err := glfw.Init(); err != nil {
		log.Fatalln("failed to initialize glfw:", err)
	}
	defer glfw.Terminate()

	// Request an OpenGL 4.1 core profile context (works on macOS too)
	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(winWidth, winHeight, "Artificial Life 3D (Go + OpenGL)", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()
	if err := gl.Init(); err != nil {
		panic(err)
	}

	version := gl.GoStr(gl.GetString(gl.VERSION))
	fmt.Println("OpenGL version", version)

	// OpenGL state
	gl.Enable(gl.DEPTH_TEST)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)
	gl.Enable(gl.PROGRAM_POINT_SIZE)

	// Build shaders and program
	prog, err = newProgram(vertexShaderSrc, fragmentShaderSrc)
	if err != nil {
		panic(err)
	}

	// Create particle data
	particles = make([]Particle, nParticles)
	initParticles()

	// Create buffers & VAO
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)

	gl.GenBuffers(1, &vboPos)
	gl.BindBuffer(gl.ARRAY_BUFFER, vboPos)
	gl.BufferData(gl.ARRAY_BUFFER, nParticles*3*4, nil, gl.DYNAMIC_DRAW)
	gl.EnableVertexAttribArray(0)
	gl.VertexAttribPointer(0, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))

	gl.GenBuffers(1, &vboCol)
	gl.BindBuffer(gl.ARRAY_BUFFER, vboCol)
	gl.BufferData(gl.ARRAY_BUFFER, nParticles*3*4, nil, gl.DYNAMIC_DRAW)
	gl.EnableVertexAttribArray(1)
	gl.VertexAttribPointer(1, 3, gl.FLOAT, false, 0, gl.PtrOffset(0))

	gl.BindVertexArray(0)

	// Input callbacks
	window.SetCursorPosCallback(mouseMove)
	window.SetMouseButtonCallback(mouseButton)
	window.SetScrollCallback(scrollCallback)

	// Main loop
	prev := time.Now()
	for !window.ShouldClose() {
		now := time.Now()
		dt := now.Sub(prev).Seconds()
		prev = now

		updateParticles(dt)
		render(window)
		window.SwapBuffers()
		glfw.PollEvents()
	}
}

func initParticles() {
	// initialize particles inside a cubic region
	for i := 0; i < nParticles; i++ {
		x := (rand.Float64()*2 - 1) * 300.0
		y := (rand.Float64()*2 - 1) * 300.0
		z := (rand.Float64()*2 - 1) * 300.0
		particles[i].Pos = mgl32.Vec3{float32(x), float32(y), float32(z)}
		particles[i].Vel = mgl32.Vec3{0, 0, 0}
		// color groups: yellow, red, green-ish mixes
		r := float32(rand.Float64()*0.6 + 0.4)
		g := float32(rand.Float64()*0.6 + 0.1)
		b := float32(rand.Float64() * 0.3)
		particles[i].Col = mgl32.Vec3{r, g, b}
	}
}

func updateParticles(dt float64) {
	// Define the target radius for the sphere
	const sphereRadius = 150.0
	// Strength of the spring-like force (higher = faster settling, more rigid)
	const springStrength = 50.0
	// Heavy damping to quickly stop all movement once the particles are in position
	const dampingFactor = 0.05

	for i := 0; i < nParticles; i++ {
		a := &particles[i]
		fx, fy, fz := 0.0, 0.0, 0.0

		// Vector from the origin (0,0,0) to the particle's position
		pos := a.Pos
		distFromCenter := float64(pos.Len())

		if distFromCenter > 0.0 {
			// Normalize the position vector to get the direction of the force
			normDir := pos.Normalize()

			// --- Spring-like Force Calculation (Hooke's Law concept) ---
			// The force is proportional to the difference between the current distance
			// and the desired sphereRadius.
			// F = -k * (x - L), where x is distFromCenter and L is sphereRadius
			forceMag := springStrength * (distFromCenter - sphereRadius)

			// The resulting force vector points toward (if dist > radius) or away (if dist < radius)
			fx = float64(-normDir.X()) * forceMag
			fy = float64(-normDir.Y()) * forceMag
			fz = float64(-normDir.Z()) * forceMag
		}

		// --- Update Velocity and Position ---

		// Apply the calculated force (acceleration)
		a.Vel = a.Vel.Add(mgl32.Vec3{float32(fx * dt), float32(fy * dt), float32(fz * dt)})

		// Apply heavy damping to make the movement quickly stop
		a.Vel = a.Vel.Mul(float32(dampingFactor))

		// Update position
		a.Pos = a.Pos.Add(a.Vel.Mul(float32(dt * 60)))

		// **Remove all boundary checks** (e.g., if a.Pos.X() > 350) as the central force
		// now handles keeping the particles contained.
	}
}

func render(window *glfw.Window) {
	gl.ClearColor(0, 0, 0, 1)
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	gl.UseProgram(prog)

	// build view/proj
	proj := mgl32.Perspective(float32(mgl32.DegToRad(45.0)), float32(winWidth)/float32(winHeight), 0.1, 5000.0)
	cam := cameraMatrix()
	vp := proj.Mul4(cam)
	loc := gl.GetUniformLocation(prog, gl.Str("uVP\x00"))
	gl.UniformMatrix4fv(loc, 1, false, &vp[0])

	// update position buffer
	posData := make([]float32, 0, nParticles*3)
	colData := make([]float32, 0, nParticles*3)
	for i := 0; i < nParticles; i++ {
		p := particles[i]
		posData = append(posData, p.Pos.X(), p.Pos.Y(), p.Pos.Z())
		colData = append(colData, p.Col.X(), p.Col.Y(), p.Col.Z())
	}

	gl.BindBuffer(gl.ARRAY_BUFFER, vboPos)
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, len(posData)*4, gl.Ptr(posData))
	gl.BindBuffer(gl.ARRAY_BUFFER, vboCol)
	gl.BufferSubData(gl.ARRAY_BUFFER, 0, len(colData)*4, gl.Ptr(colData))

	// draw
	gl.BindVertexArray(vao)
	gl.PointSize(pointSize)
	gl.DrawArrays(gl.POINTS, 0, int32(nParticles))
	gl.BindVertexArray(0)
}

func cameraMatrix() mgl32.Mat4 {
	// position from spherical coordinates
	x := float32(distance * math.Cos(azimuth) * math.Cos(elevation))
	y := float32(distance * math.Sin(elevation))
	z := float32(distance * math.Sin(azimuth) * math.Cos(elevation))
	eye := mgl32.Vec3{x, y, z}
	center := mgl32.Vec3{0, 0, 0}
	up := mgl32.Vec3{0, 1, 0}
	view := mgl32.LookAtV(eye, center, up)
	return view
}

// Input callbacks
func mouseMove(w *glfw.Window, xpos float64, ypos float64) {
	if dragging {
		dx := xpos - lastX
		dy := ypos - lastY
		azimuth -= dx * 0.005
		elevation -= dy * 0.005
		if elevation > 1.4 {
			elevation = 1.4
		}
		if elevation < -1.4 {
			elevation = -1.4
		}
	}
	lastX = xpos
	lastY = ypos
}

func mouseButton(w *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
	if button == glfw.MouseButton1 {
		if action == glfw.Press {
			dragging = true
		} else if action == glfw.Release {
			dragging = false
		}
	}
}

func scrollCallback(w *glfw.Window, xoff float64, yoff float64) {
	distance -= yoff * 30
	if distance < 200 {
		distance = 200
	}
	if distance > 3000 {
		distance = 3000
	}
}

// -------------------- shader helpers --------------------

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

// -------------------- GLSL Shaders --------------------

var vertexShaderSrc = `
#version 410 core
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
uniform mat4 uVP;
out vec3 vColor;
void main() {
    vColor = inColor;
    gl_Position = uVP * vec4(inPosition, 1.0);
    float size = 6.0 / (gl_Position.z + 1.0);
    gl_PointSize = clamp(size * 20.0, 2.0, 24.0);
}
` + "\x00"

var fragmentShaderSrc = `
#version 410 core
in vec3 vColor;
out vec4 fragColor;
void main() {
    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    if (dot(coord, coord) > 1.0) discard;
    fragColor = vec4(vColor, 1.0);
}
` + "\x00"
