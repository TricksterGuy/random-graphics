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