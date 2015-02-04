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