#<- Begin File graphics_test.rb ->
require 'ruby-processing'
#<- Begin File geometry.rb ->

#<- End File geometry.rb ->

#<- Begin File draw.rb ->
module Draw

  def draw_point(point, thick = 2)

    ellipse(point.x, point.y, thick, thick)

  end

  def draw_vector(vector, point = Point::ORIGIN)

    line(point.x, point.y, point.x + vector.x, point.y + vector.y)

  end

  def draw_arrow(vector, point = Point::ORIGIN)

    draw_vector(vector, point)

    final = point + vector

    mag = [height / 50, vector.length / 10].min

    vector = vector.unit

    a = vector.rotate(-Math::PI / 8) * mag

    b = vector.rotate(Math::PI / 8) * mag

    draw_vector(a, final - a)

    draw_vector(b, final - b)

  end

  def draw_polygon(polygon, hash ={})

    hash.key?(:open) ? begin_shape(hash[open]) : begin_shape

      polygon.each do |point|

        vertex(point.x, point.y)

      end

    hash.key?(:close) ? end_shape(hash[close]) : end_shape(2)

  end

  def draw_smooth_polygon(polygon, refine = 5, hash={})

    p = polygon.dup

    vertices = p.vertices.dup

    refine.times do

      copy = vertices.dup

      size = copy.size

      copy.each_index do |i|

        prev, cur, nex, nnex = copy.values_at((i - 1) % size, i, (i + 1) % size, (i + 2) % size)

        vertices[2 * i] = cur

        vertices[2 * i + 1] = Point.towards(Point.towards(prev, 9.0/8, cur) , 0.5, Point.towards(nnex, 9.0/8, nex))

      end

    end

    p.vertices = vertices

    draw_polygon(p, hash)

  end

end



#pt L(pt A, float s, pt B) {return P(A.x+s*(B.x-A.x),A.y+s*(B.y-A.y)); };                             // Linear interpolation: A+sAB    

#pt B(pt A, pt B, pt C, float s) {return( L(L(B,s/4.,A),0.5,L(B,s/4.,C))); };                            // returns a tucked B towards its neighbors

#pt F(pt A, pt B, pt C, pt D, float s) {return( L(L(A,1.+(1.-s)/8.,B) ,0.5, L(D,1.+(1.-s)/8.,C))); };    // returns a bulged mid-edge point 
#<- End File draw.rb ->

#<- Begin File triangulation.rb ->


class Triangulation

  attr_accessor :recent

  attr_accessor :graph

  

  def initialize(triangle)

    @graph = Graph.new

    @graph.add(triangle)

    @recent = triangle

  end

  

  def triangles

    return @graph.triangles

  end

  

  def contains?(triangle)

    return @graph.contains?(triangle)

  end

  

  def opposite(point, triangle)

    opptri = @graph[triangle].detect {|neighbor| !neighbor.vertex?(point)}

    return (opptri.to_a - (triangle.to_a - [point]))[0]

  end

  

  def opposite_triangle(point, triangle)

    return @graph[triangle].detect {|neighbor| !neighbor.vertex?(point)}

  end

  

  def surrounding(point, triangle)

    queue = [triangle]

    visited = {}

    list = []

    until queue.empty?

      tri = queue.pop

      next if visited.key?(tri)

      list << tri

      visited[tri] = true

      @graph[tri].each do |neigh|

        queue << neigh if neigh.vertex?(point) and !visited.key?(neigh)

      end

    end

    return list

  end

  

  def locate(point)

    triangle = contains?(@recent) ? @recent : nil

    visited = {}

    until triangle.nil?

      break if visited.key?(triangle)

      visited[triangle] = true

      return triangle if point.in_triangle?(*triangle)

      max = 0

      corner = nil

      triangle.each do |p|

        if point.distance(p) > max

          corner = p

          max = point.distance(p)

        end

      end

      triangle = opposite_triangle(corner, triangle)

    end



    triangles.each do |tri|

      return tri if point.in_triangle?(*tri)

    end

    return nil

  end

  

  def add(point)

    tri = locate(point)

    return if tri.nil? #the point is on one of the triangles or out of bounds

    cav = cavity(point, tri)

    @recent = update(point, cav)

  end

  

  def cavity(point, triangle)

    encroached = {}

    checking = [triangle]

    marked = {triangle => true}

    until checking.empty?

      triangle = checking.shift

      next if triangle.delaunay?(point)

      

      encroached[triangle] = true

      @graph[triangle].each do |neighbor|

        next if marked.key?(neighbor)

        marked[neighbor] = true

        checking << neighbor

      end

    end

    return encroached.keys

  end

  

  def update(point, cavity)

    boundary = {}

    the_triangles = []

    cavity.each do |triangle|

      the_triangles.concat(@graph[triangle])

      triangle.each do |vertex|

        facet = triangle.opposite(vertex)

        boundary.key?(facet) ? boundary.delete(facet) : boundary[facet] = true

      end

    end

    the_triangles -= cavity

    cavity.each {|triangle| @graph.remove(triangle)}

    new_triangles = {}

    boundary.each_key do |pts|

      a, b, c = point, pts

      tri = Triangle.new(point, *pts)

      @graph.add(tri)

      new_triangles[tri] = true

    end

    the_triangles.concat(new_triangles.keys)

    new_triangles.each_key do |triangle|

      the_triangles.each do |other|

        if triangle.neighbor?(other)

          @graph.add(triangle, other)

        end

      end

    end

    return new_triangles.keys[0]

  end

  

  def each

    @graph.triangles.each {|t| yield t}

  end

end



class Graph

  attr_accessor :data

  

  def initialize

    @data = Hash.new([])

  end

  

  def [](triangle)

    return @data[triangle]

  end

  

  def add(triangle, neighbor = nil)

    if neighbor.nil?

      return if @data.key?(triangle)

      @data[triangle] = []

    else

      @data[triangle] << neighbor if !@data[triangle].include?(neighbor)

      @data[neighbor] << triangle if !@data[neighbor].include?(triangle)

    end

  end

  

  

  def remove(triangle, neighbor = nil)

    unless neighbor

      neighbors = @data[triangle]

      neighbors.each do |neigh|

        @data[neigh].delete(triangle)

      end

      @data.delete(triangle)

    else

      @data[triangle].delete(neighbor)

      @data[neighbor].delete(triangle)

    end

  end

  

  def [](triangle)

    return @data[triangle]

  end

  

  def triangles

    return @data.keys

  end

  

  def contains?(triangle)

    return @data.key?(triangle)

  end

end
#<- End File triangulation.rb ->

#<- Begin File mesh.rb ->
class Mesh

  attr_accessor :vertices

  attr_accessor :incidences

  attr_accessor :opposites

  attr_accessor :triangles

  

  def initialize(triangulation, polygon = nil)

    @triangles = []

    @opposites = []

    @vertices = {}

    vertices = {}

    @incidences = []

    waiting = {}

    i = 0

    corner = 0

    triangulation.each do |triangle|

      # If its bad don't do it please

      next unless triangle.all? {|p| p.in_bounds?(800, 600)}

      next if !(polygon.nil? || triangle.segments.all? {|seg| seg.in_polygon?(polygon)})

      @triangles << triangle

      triangle.clockwise.each do |point|

        vertices[point] = (i += 1) if !vertices.key?(point)

        @incidences << vertices[point]

        opposite = triangulation.opposite(point, triangle)

        opp_triangle = triangulation.opposite_triangle(point, triangle)

        # Case where opposite is pointing into outer triangle

        if opposite.in_bounds?(800, 600)

          # Case where triangle is invalid

          if opp_triangle.segments.all? {|seg| seg.in_polygon?(polygon)}

            waiting[corner] = [opposite, opp_triangle]

          else

            @opposites[corner] = corner

          end

        else

          @opposites[corner] = corner

        end

        corner += 1

      end

    end

    # Calculate Corners

    waiting.each do |c, opposing|

      opposite, triangle = opposing

      id = @triangles.index(triangle)

      vertex_id = vertices[opposite]

      sub = @incidences[(3*id)..(3*id+2)]

      @opposites[c] = sub.index(vertex_id) + 3 * id

    end

    # Invert vertices hash to get vertex table

    @vertices = vertices.invert

  end

  

  def triangle(c)

    return c / 3

  end

  

  def after(c)

    return 3 * triangle(c) + (c + 1) % 3

  end

  

  def before(c)

    return 3 * triangle(c) + (c - 1) % 3

  end

  

  def vertex(c)

    return incidences[c]

  end

  

  def point(c)

    return vertices[vertex(c)]

  end

  

  def border?(c)

    return opposites[c] == c

  end

  

  def opposite(c)

    return opposites[c]

  end

  

  def left(c)

    return opposite(before(c))

  end

  

  def right(c)

    return opposite(after(c))

  end

  

  def swing(c)

    return before(opposite(before(c)))

  end

end
#<- End File mesh.rb ->

#<- Begin File point.rb ->
class Point
  include Comparable

  
  Epsilon = 1e-7
  

  attr_accessor :x

  attr_accessor :y

  

  def self.from(point)

    return self.new(point.x, point.y)

  end

  

  def self.translated(point, vector)

    return self.new(point.x + vector.x, point.y + vector.y)

  end

  

  def self.direction(point, distance, vector)

    return self.new(point.x + distance * vector.x, point.y * distance * vector.y)

  end

  

  def self.towards(start, distance, finish)

    return self.new(start.x + distance * (finish.x - start.x), start.y + distance * (finish.y - start.y))

  end



  def initialize(x = 0, y = 0)

    @x = x

    @y = y

  end
  
  def <=>(other)
    if (x <=> other.x) != 0
      return x <=> other.x
    else
      return y <=> other.y
    end
  end
  
  def ==(other)
    return x == other.x && y == other.y
  end

    

  def +(vector)

    return translate(vector.x, vector.y)

  end

  

  def -(vector)

    return self + -vector

  end

  

  def set(*args)

    if args.size == 1

      @x, @y = args[0].x, args[0].y

    else

      @x, @y = args

    end

  end

  

  def translate!(dx, dy)

    @x += dx

    @y += dy

  end

  

  def translate(dx, dy)

    return Point.new(x + dx, y + dy)

  end

  

  def towards!(s, v)

    @x += s * v.x

    @y += s * v.y

  end

  

  def towards(s, v)

    return Point.new(x + s * v.x, y + s * v.y)

  end

  

  def translate_towards!(s, p)

    towards!(s, to(p))

  end

  

  def translate_towards(s, p)

    towards(s, to(p))

  end
  
  def move_towards!(d, p)
    vec = to(p)
    vec.unit!
    towards!(d, vec)
  end
  
  def move_towards(d, p)
    vec = to(p)
    vec.unit!
    towards(d, vec)
  end

  

  def rotate!(a, point = ORIGIN)

    dx = x - point.x

    dy = y - point.y

    c = Math.cos(a)

    s = Math.sin(a)

    @x = point.x + c * dx - s * dy

    @y = point.y + s * dx + c * dy

  end

  

  def rotate(a, point = ORIGIN)

    dx = x - point.x

    dy = y - point.y

    c = Math.cos(a)

    s = Math.sin(a)

    x = point.x + c * dx - s * dy

    y = point.y + s * dx + c * dy

    return Point.new(x, y)

  end

  

  def scale!(s, t = s)

    @x *= s

    @y *= t

  end

  

  def scale(s, t = s)

    return Point.new(@x * s, @y * t)

  end

  

  def projection!(p, q) 

    vec = p.to(self)

    line = p.to(q)

    a = vec.dot(line)

    b = line.distance

    p.towards!(a/b, q)

  end

  

  def projection(p, q) 

    vec = p.to(self)

    line = p.to(q)

    a = vec.dot(line)

    b = line.distance

    return p.towards(a/b, q)

  end

  

  def distance(p)

    return Math.sqrt((x - p.x) ** 2 + (y - p.y) ** 2)

  end

  

  def projects?(p, q) 

    vec = p.to(self)

    line = p.to(q)

    a = vec.dot(line)

    b = line.distance

    return (0 < a) && (a < b)

  end

  

  def to(other)

    return Vector.points(self, other)

  end

  

  def translation

    return Vector.new(x, y)

  end

  

  def right_of?(a, b)

    ab = a.to(b)

    bc = b.to(self)

    ab.left!

    return ab.dot(bc) > 0

  end

    

  def in_circle?(p, r)

    return distance(p) < r

  end

  

  def in_triangle?(a, b, c)

    bc = right_of?(b, c)

    ca = right_of?(c, a)

    ab = right_of?(a, b)

    return bc&&ca&&ab || !bc&&!ca&&!ab

  end

  

  def between?(a, b)

    ca = to(a)

    cb = to(b)

    between = ca.dot(cb) <= 0

    ca.left!

    parallel = ca.dot(cb).abs <= Epsilon

    return between && parallel

  end

  

  def in_bounds?(width, height)

    return in_rect?(0, 0, width, height)

  end

  

  def in_rect?(x, y, width, height)

    fx = x + width

    fy = y + height

    return @x >= x && @x < fx && @y >= y && @y < fy

  end
  
  def on_polygon?(polygon)
    polygon.each_segment do |segment|
      return true if segment.contains?(self)
    end
    return false
  end
  
  def in_polygon?(polygon, width = 800, height = 600)
    count = 0
    x2 = rand(width) + width
    y2 = rand(height) + height
    ray = Segment.new(self, Point.new(x2, y2))
    polygon.each_segment do |segment|
      return false if segment.contains?(self)
      count += 1 if ray.cross?(segment)
    end
    return count % 2 != 0
  end

  
  def in_circumcircle?(a, b, c)
    center = a.circle(b, c)
    radius = a.distance(center)
    return in_circle?(center, radius)
  end

  def circle(b, c) 
    ab =  self.to(b)
    ab_length = ab.dot(ab)
    ac =  self.to(c) 
    ac.left!
    ac_length = ac.dot(ac)
    d = 2 * ab.dot(ac)
    ab.left!
    ab.scale!(-ac_length)
    ac.scale!(ab_length)
    ab += ac
    ab.scale!(1.0 / d)
    return self + ab
  end
  

  def to_s

    return "(#{x}, #{y})"

  end

  
  def inspect
    return to_s
  end

  ORIGIN = Point.new(0, 0)

end
#<- End File point.rb ->

#<- Begin File vector.rb ->
class Vector

  

  attr_accessor :x

  attr_accessor :y

  

  def self.from(vector)

    return self.new(vector.x, vector.y)

  end

  

  def self.points(start, finish)

    return self.new(finish.x - start.x, finish.y - start.y)

  end

  

  def initialize(x = 0, y = 0)

    @x = x

    @y = y

  end

  

  def set(x, y)

    @x = x

    @y = y

  end

  

  def +(vector)

    Vector.new(x + vector.x, y + vector.y)

  end

  

  def -(vector)

    self + -vector

  end

  

  def *(scalar)

    scale(scalar)

  end

  

  def /(scalar)

    scale(1.0/scalar)

  end

  

  def -@

    return Vector.new(-x, -y)

  end

    

  def dot(vec)

    return x * vec.x + y * vec.y

  end

  

  def cross(vec)

    return x * vec.y - vec.x * y

  end

  

  def length

    return Math.sqrt(x ** 2 + y ** 2)

  end

  

  def normalize!

    scale!(1.0 / length)

  end

  

  def normalize

    return scale(1.0 / length)

  end

  

  alias unit! normalize!

  alias unit normalize

  

  def scale!(s, t = s)

    @x *= s

    @y *= t

  end

  

  def scale(s, t = s)

    return Vector.new(x * s, y * t)

  end

  

  def left!

    @x, @y = -y, x

  end

  

  def left

    return Vector.new(-y, x)

  end

  

  def rotate!(angle)

    @x, @y = x * Math.cos(angle) - y * Math.sin(angle), x * Math.sin(angle) + y * Math.cos(angle)

  end

  

  def rotate(angle)

    nx = x * Math.cos(angle) - y * Math.sin(angle)

    ny = x * Math.sin(angle) + y * Math.cos(angle)

    return Vector.new(nx, ny)

  end

  

  def reflect!(normal)

    scalar = -2 * dot(normal)

    @x += scalar * normal.x

    @y += scalar * normal.y

  end

  

  def reflect(normal)

    scalar = -2 * dot(normal)

    return Vector.new(x + scalar * normal.x, y + scalar * normal.y)

  end

  

  def angle

    return Math.atan2(y, x)

  end



  def to_s

    return "<#{x}, #{y}>"

  end

end
#<- End File vector.rb ->

#<- Begin File polygon.rb ->
Infinity = 1 / 0.0

Epsilon = 1e-7



class Polygon

  include Enumerable

  attr_accessor :vertices

  

  def self.n_gon(vertices, center, radius)

    points = Array.new(vertices) {Point.new}

    points.each_with_index do |point, i|

      angle = Math::PI * (1 - i * 4.0 / vertices) / 2

      point.x = center.x + radius * Math.cos(angle)

      point.y = center.y + radius * Math.sin(angle)

    end

    return Polygon.new(*points)

  end

  

  def initialize(*args)

    @vertices = args

  end

  

  def [](index)

    return @vertices[index]

  end

  

  def add(point)

    @vertices << point

  end

  

  def <<(point)

    @vertices << point

    return self

  end

  

  def translate!(x, y)

    @vertices.each {|point| point.translate!(x, y)}

  end

  

  def translate(x, y)

    p = Polygon.new(*@vertices)

    p.vertices.collect! {|point| point.translate(x, y)}

    return p

  end



  def each

    @vertices.each {|point| yield point}

  end

  

  def each_edge

    @vertices.each_index do |i|

      yield @vertices[i], @vertices[(i + 1) % @vertices.size]

    end

  end

  

  def each_segment

    each_edge {|a, b| yield Segment.new(a, b)}

  end

  

  def segments

    ary = []

    each_segment {|s| ary << s}

    return ary

  end

  

  def size

    return @vertices.size

  end

  

  def closest_point(point, max = Infinity)

    min = Infinity

    chosen = @vertices[0]

    @vertices.each do |vertex|

      if vertex.distance(point) < min

        chosen = vertex

        min = vertex.distance(point)

      end

    end

    

    return min > max ? nil : chosen

  end

  

  def area

    a = 0

    each_edge do |p1, p2|

      a += p1.translation.cross(p2.translation)

    end

    return a.abs / 2

  end

  

  def perimeter

    a = 0

    each_edge do |p1, p2|

      a += p1.distance(p2)

    end

    return a

  end

  

  def convex_hull

    vertices = []

    p = @vertices.min

    @vertices.sort!

    start = p

    vertices << start

    begin

      n = -1

      dist = 0

      @vertices.each_with_index do |point, i|

        next if point == p 

        n = i if n == -1

        cross = (p.to(point)).cross(p.to(@vertices[n]))

        d = p.distance(point)

        if cross < 0

          n = i

          dist = d

        elsif cross == 0 and d > dist

          dist = d

          n = i

        end

      end

      p = @vertices[n]

      vertices << p

    end until start == p

    vertices.pop

    vertices.delete_at(1) if vertices[1].between?(vertices[0], vertices[2])

    vertices.delete_at(-1) if vertices[-1].between?(vertices[0], vertices[-2])

    return vertices

  end



  def mirror(width = 800)

    polygon = Polygon.new

    each do |p|

      polygon.add(Point.new(width - p.x, p.y))

    end

    return polygon

  end

  

  def closest(point)

    min = Infinity

    close = @vertices[0]

    @vertices.each do |p|

      if min > point.distance(p)

        close = p

        min = point.distance(p)

      end

    end

    return close

  end

end


#<- End File polygon.rb ->

#<- Begin File line.rb ->
class Line

  attr_accessor :start

  attr_accessor :finish

  

  def self.points(x1, y1, x2, y2)

    return Line.new(Point.new(x1, y1), Point.new(x2, y2))

  end

  

  def initialize(start, finish)

    @start = start

    @finish = finish

  end

  

  def contains?(point)

    to_start = point.to(start)

    return to_start.cross(vector) == 0

  end

  

  def cross?(line)

    return vector.cross(line.vector) != 0

  end

  

  def intersect(line)

    a1 = y2 - y1

    a2 = line.y2 - line.y1

    b1 = x2 - x1

    b2 = line.x2 - line.x1

    c1 = a1 * x2 + b1 * y1

    c2 = a2 * x2 + b2 * y2

    det = (a1 * b2 - a2 * b1).to_f

    x = (b2 * c1 - b1 * c2) / det

    y = (a1 * c2 - a2 * c1) / det

    return Point.new(x, y)

  end

  

  def vector

    return start.to(finish)

  end

end
#<- End File line.rb ->

#<- Begin File ray.rb ->
class Ray < Line

end
#<- End File ray.rb ->

#<- Begin File segment.rb ->
class Segment < Line

  def contains?(point)

    return point.between?(start, finish)

  end

  

  def cross?(segment)

    right_of?(segment.start) ^ right_of?(segment.finish) && segment.right_of?(start) ^ segment.right_of?(finish)

  end

  

  def right_of?(point)

    return !point.right_of?(start, finish)

  end

  

  def contains?(point)

    return point.between?(start, finish)

  end

  

  def ==(other)

    return [start, finish].include?(other.start) && [start, finish].include?(other.finish)

  end

  

  def in_polygon?(polygon)

    return false unless (polygon.vertices.include?(start) || start.in_polygon?(polygon))

    return false unless (polygon.vertices.include?(finish) || finish.in_polygon?(polygon))

    mid = Point.towards(start, 0.5, finish)

    return mid.on_polygon?(polygon) || mid.in_polygon?(polygon) 

  end

  

  def inspect

    return start.inspect + "->" + finish.inspect

  end

end
#<- End File segment.rb ->

#<- Begin File triangle.rb ->
class Triangle

  include Enumerable

  

  attr_accessor :a, :b, :c

  

  def initialize(a, b, c)

    @a = a

    @b = b

    @c = c

  end

  

  def get_vertex(*args)

    return (clockwise - args).first

  end

  

  def vertex?(point)

    return a == point || b == point || c == point

  end

  

  def neighbor?(triangle)

    count = 0

    [a, b, c].each do |vertex|

      count += 1 if triangle.vertex?(vertex)

    end

    return count == 2

  end

  

  def opposite(point)

    return [a, b, c] - [point]

  end

  

  def each

    [a, b, c].each {|p| yield p}

  end

  

  def clockwise?

    return normal < 0

  end

  

  def delaunay?(point)

    return !in_circumcircle?(point)

  end

  

  def in_circumcircle?(point)

    return point.in_circumcircle?(a, b, c)

  end

 

  def circumcircle

    return a.circle(b, c)

  end

  

  def sanitize!

    if !clockwise?

      @a, @b, @c = [a, c, b]

    end

  end

  

  def clockwise

    return clockwise? ? to_a : [a, c, b]

  end



  def normal

    ab = a.to(b)

    ac = a.to(c)

    return ab.cross(ac) 

  end

  

  def to_a

    return [a, b, c]

  end

  

  def segments

    return [Segment.new(a, b), Segment.new(b, c), Segment.new(c, a)]

  end

  

  def eql?(o)

    return (to_a | o.to_a).size == 3

  end

end
#<- End File triangle.rb ->

class Sketch < Processing::App
  include Draw
  def setup
    @show_help = true
    @smooth = false
    @mode = 'c'
    @color = false
    @mat = false
    frame_rate(64)
    font = load_font("Courier-14.vlw")
    text_font(font, 14)
    @polygon = Polygon.n_gon(16, Point.new(400, 300), 250)
    @points = []
  end
  
  def draw
    background(color(204, 204, 204))
    @show_help ? show_help : actions
  end
  
  def key_pressed
    case key
    when 32
      @show_help = !@show_help
    when ?>
      double_vertices
    when ?<
      halfen_vertices
    when ?.
      double_inner_point
    when ?,
      halfen_inner_point
    when ?c
      @points.clear
    when ?C
      @polygon.vertices = @polygon.convex_hull
    when ?g
      @smooth = !@smooth
    when ?e
      @mode = 'c'
    when ?t
      # Rebuild Triangulation if was in curve
      init_triangulation if @mode == 'c'
      @mode = 't'
    when ?m  
      # Build Triangulation first
      init_triangulation if @mode == 'c'
      @mode = 'm'
      @mesh = Mesh.new(@triangulation, @polygon)
      @corner = 0
    when ?M
      @mat = !@mat
    when ?x
      @mode = 'm'
      mirror_mesh
      @corner = 0
    when ?v
      # Build Triangulation first
      init_triangulation if @mode == 'c'
      @mode = 'v'
    when ?R
      resample
    when ?I
      @color = !@color
    when ?W
      save_data
    when ?L
      load_data
    when ?l
      @corner = @mesh.left(@corner)
    when ?r
      @corner = @mesh.right(@corner)
    when ?s
      @corner = @mesh.swing(@corner)
    when ?n
      @corner = @mesh.after(@corner)
    when ?p
      @corner = @mesh.before(@corner)
    when ?o
      @corner = @mesh.opposite(@corner)
    end
  end
   
  def mouse_pressed
    mouse_point = Point.new(mouse_x, mouse_y)
    @point = @polygon.closest_point(mouse_point, 4)
    if key_pressed? and key == ?d and @point.nil?
      closest = @polygon.closest(mouse_point)
      index = @polygon.vertices.index(closest)
      @polygon.vertices.insert(index + 1, mouse_point)
    elsif @point.nil? and mouse_point.in_polygon?(@polygon)
      @points << mouse_point
    end
  end
  
  def mouse_released
    @point = nil
  end
  
  def actions
    case @mode
    when 't'
      draw_triangulation
    when 'm'      
      draw_mesh
      stroke(color(255, 255, 255))
      draw_point(@mesh.point(@corner), 10)
      stroke(color(0, 0, 0))
    when 'v'
      draw_voronoi
    else
      if mouse_pressed? && !key_pressed? and !@point.nil?
        @point.translate!(mouse_x - pmouse_x, mouse_y - pmouse_y)
      end
      @smooth ? draw_smooth_polygon(@polygon) : draw_polygon(@polygon)
    end
    @polygon.each {|point| draw_point(point, 4)}
    @points.each {|point| draw_point(point, 2)}
    draw_mat if (@mat && !@triangulation.nil?)
  end
  
  def show_help
    @line = 1
    fill(color(0, 0, 128))
    scribe("Project 5 (Bulge) by Brandon Whitehead")
    scribe("")
    scribe("To insert a point hold d and click in/on the polygon")
    scribe("To modify vertex count press < or >")
    scribe("To show the Delaunay triangulation press t")
    scribe("To show the MAT sample points press M")
    scribe("To add/remove random points in the polygon press , or .")
    scribe("To Laplace smooth the points press S (not functional)")
    scribe("To bulge into 3D press b (not functional)")
    scribe("To save the (bulged) mesh to a file press W")
    scribe("To lo" + "ad a (bulged) mesh from file press L") # ehh can't say load :/
    scribe("To animate the 3d mesh press a (not functional)")
    scribe("To compute the isolation measure press I")
    scribe("")
    scribe("To turn on smoothing press g")
    scribe("To compute the convex hull of the polygon press C")
    scribe("To resample the polygon press R")
    scribe("To clear all points in the interior of the polygon press c")
    scribe("")
    scribe("Modes")
    scribe("Press e for curve editing mode")
    scribe("Press t for triangulation viewing mode")
    scribe("Press v for viewing voronoi diagram")
    scribe("Press m for mesh viewing mode")
    scribe("Press x for mirror mesh (2d) viewing")
    scribe("Press o/s/n/p/l/r while in mesh mode to navigate mesh (look for white circle)")
    scribe("")
    scribe("Press Space to hide this help text")
    no_fill
  end
  
  def scribe(text)
    text(text, 20, 20 * @line)
    @line += 1
  end
  
  def init_triangulation
    max = 10000
    triangle = Triangle.new(Point.new(-max, -max), Point.new(max, -max), Point.new(0, max))
    @triangulation = Triangulation.new(triangle)
    @points.each {|p| @triangulation.add(p)}
    @polygon.each {|p| @triangulation.add(p)}
  end
  
  def mirror_mesh
    polygon = @polygon.mirror
    max = 10000
    triangle = Triangle.new(Point.new(-max, -max), Point.new(max, -max), Point.new(0, max))
    @triangulation = Triangulation.new(triangle)
    @points.each {|p| @triangulation.add(Point.new(800 - p.x, p.y))}
    polygon.each {|p| @triangulation.add(p)}
    @mesh = Mesh.new(@triangulation, polygon)
  end
  
  def double_vertices
    vertices = []
    @polygon.each_edge do |a, b|
      vertices << a
      vertices << Point.towards(a, 0.5, b)
    end
    @polygon.vertices = vertices
  end
    
  def halfen_vertices
    if @polygon.size / 2 >= 3
      vertices = []
      @polygon.each_with_index do |a, i|
        vertices << a if i & 1 == 0
      end
      @polygon.vertices = vertices
    end
  end
  
  def double_inner_point
    number = @points.empty? ? 1 : @points.size
    number.times do
      point = Point.new(rand(800), rand(600))
      point = Point.new(rand(800), rand(600)) while !point.in_polygon?(@polygon)
      @points << point
    end
  end
  
  def halfen_inner_point
    number = @points.empty? ? 0 : @points.size / 2
    number.times do 
      @points.delete_at(rand(@points.size))
    end
  end
  
  def resample
    length = @polygon.perimeter
    dist = length / @polygon.size
    remain = dist
    remain_length = 0
    vertices = 1
    point = Point.from(@polygon[0])
    k = 0
    nk = 0
    while vertices < @polygon.size
      nk = (k + 1) % @polygon.size
      remain_length = point.distance(@polygon[nk])
      if remain < remain_length
        point.move_towards!(remain, @polygon[nk])
        @polygon[vertices].set(point)
        vertices += 1
        remain_length -= remain
        remain = dist
      else
        remain -= remain_length
        point = Point.from(@polygon[nk])
        k += 1
      end
    end
  end
  
  def draw_triangulation
    return if @triangulation.nil?
    triangles = @triangulation.triangles
    triangles = triangles.select {|triangle| triangle.all? {|p| p.in_bounds?(width, height)}}
    triangles.each do |triangle|
      begin_shape
        triangle.each do |point|
          vertex(point.x, point.y)
        end
      end_shape(CLOSE)
    end
  end
  
  def draw_voronoi
    done = {}
    @triangulation.triangles.each do |triangle|
      # If its bad don't do it please
      next unless triangle.all? {|p| p.in_bounds?(800, 600)}
      triangle.clockwise.each do |point|
        next if done.key?(point)
        done[point] = true
        surround = @triangulation.surrounding(point, triangle)
        polygon = Polygon.new
        i = 0
        surround.each do |tri|
          polygon.add(tri.circumcircle)
          i += 1
        end
        draw_polygon(polygon)
      end
    end
  end
  
  def draw_mat
    @triangulation.triangles.each do |triangle|
      # If its bad don't do it please
      next unless triangle.all? {|p| p.in_bounds?(800, 600)}
      draw_point(triangle.circumcircle, 2)
    end
  end
  
  def draw_mesh
    if @color
      fill(color(0, 25, 255)) 
      normals = @mesh.triangles.dup
      normals.collect! {|t| t.normal.abs}
      maxn = normals.max * 1.2
    end
    @mesh.triangles.each do |triangle|
      fill(color(0, 25, 255 - 255 * triangle.normal.abs / maxn)) if @color
      begin_shape(POLYGON)
        triangle.each do |point|
          vertex(point.x, point.y)
        end
      end_shape(CLOSE)
    end
    no_fill
  end
  
  def save_data
    file = File.new('mesh.gph', 'w')
#    Marshal.dump(@polygon, file)
#    p @polygon
#    Marshal.dump(@points, file)
#    p @points
#    Marshal.dump(@triangulation, file)
#    p @triangulation
#    Marshal.dump(@mesh, file)
#    p @mesh
    Marshal.dump([@polygon, @points, @triangulation, @mesh], file)
    file.close
  end
  
  def load_data
    file = File.new('mesh.gph', 'rb')
#    @polygon = Marshal.load(file)
#    p @polygon
#    @points = Marshal.load(file)
#    p @points
#    @triangulation = Marshal.load(file)
#    p @triangulation
#    @mesh = Marshal.load(file)
#    p @mesh
    @polygon, @points, @triangulation, @mesh = Marshal.load(file)
    file.close
  end
end

Sketch.new(:width => 800, :height => 600, :title => "Test")
#<- End File graphics_test.rb ->