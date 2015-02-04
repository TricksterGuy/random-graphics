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