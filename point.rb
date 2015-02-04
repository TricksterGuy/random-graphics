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