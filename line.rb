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