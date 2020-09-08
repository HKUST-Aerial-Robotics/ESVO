#ifndef ESVO_CORE_CONTAINER_SMARTGRID_H
#define ESVO_CORE_CONTAINER_SMARTGRID_H

#include <cstdlib>
#include <memory>
#include <list>
#include <vector>
#include <set>

namespace esvo_core
{
namespace container
{
template<class T>
class SmartGrid
{
  public:
  typedef std::shared_ptr<SmartGrid> Ptr;
  typedef std::list<T> gridElements;
  typedef typename gridElements::iterator iterator;

  SmartGrid();
  SmartGrid(size_t rows, size_t cols);
  virtual ~SmartGrid();
  SmartGrid<T> &operator=(const SmartGrid &rhs);

  // utils
  void resize(size_t rows, size_t cols);
  void erode(size_t radius, size_t border = 0, double ratio = 0.5);
  void dilate(size_t radius);
  void clear();
  void clean(double var_threshold, double age_threshold, double range_max, double range_min);
  void reset();

  const T &operator()(size_t row, size_t col) const;
  const T &at(size_t row, size_t col) const;

  size_t rows() const;
  size_t cols() const;

  void set(size_t row, size_t col, const T &value);
  T &get(size_t row, size_t col);
  bool exists(size_t row, size_t col);

  iterator remove(iterator &it);
  iterator begin();
  iterator end();

  size_t size() const;
  void getNeighbourhood(size_t row, size_t col, size_t radius, std::vector<T *> &neighbours);

  private:
  std::vector<std::vector<T *> *> _grid;
  gridElements _elements;
  T _invalid; //invalid element that we can return when the element doesn't exist
};

template<class T>
SmartGrid<T>::SmartGrid() = default;

template<class T>
SmartGrid<T>::SmartGrid(size_t rows, size_t cols)
{
  _grid.reserve(rows);
  for (size_t r = 0; r < rows; r++)
  {
    _grid.push_back(new std::vector<T *>());
    (*_grid.back()).resize(cols, NULL);
  }
}

template<class T>
SmartGrid<T>::~SmartGrid()
{
  for (size_t r = 0; r < rows(); r++)
    delete _grid[r];
}

template<class T>
SmartGrid<T> &
SmartGrid<T>::operator=(const SmartGrid &rhs)
{
  //copy grid elements
  _elements = rhs._elements;

  //remove the old grid
  for (size_t r = 0; r < rows(); r++)
    delete _grid[r];
  _grid.clear();

  //initialize a new NULL grid with correct size
  for (size_t r = 0; r < rhs.rows(); r++)
  {
    _grid.push_back(new std::vector<T *>());
    (*_grid.back()).resize(rhs.cols(), NULL);
  }

  //go through all the new elements and add the pointers one by one
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    (*_grid[it->row()])[it->col()] = &const_cast<T &>(*it);
    it++;
  }

  return *this;
}

template<class T>
void
SmartGrid<T>::resize(size_t rows, size_t cols)
{
  this->reset();
  _grid.reserve(rows);
  for (size_t r = 0; r < rows; r++)
  {
    _grid.push_back(new std::vector<T>());
    (*_grid.back()).resize(cols, NULL);
  }
}

template<class T>
void
SmartGrid<T>::erode(size_t radius, size_t border, double ratio)
{
  // have to save a temporary copy of the depth map
  SmartGrid gridTmp(rows(), cols());

  // first transfer all the existing elements
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    gridTmp.set(it->row(), it->col(), *it);
    it++;
  }

  size_t empty_pixel_count;
  size_t num_overall_pixel = (2 * radius + 1)*(2 * radius + 1);
//  size_t num_inner_circle_pixel = (2 * (radius - 1) + 1) * (2 * (radius - 1) + 1);

  //erode if not enough neightbours exist(within radius)
  it = gridTmp._elements.begin();
  auto it2 = _elements.begin();
  while (it != gridTmp._elements.end())
  {
    empty_pixel_count = 0;
    for (int r = it->row() - radius; r < it->row() + radius + 1; r++)
    {
      for (int c = it->col() - radius; c < it->col() + radius + 1; c++)
      {
        //check whether this location is inside the image
        if (r >= border && r < (int) rows() - border && c >= border && c < (int) cols() - border)
        {
          //check whether that element is missing
          if ((*(gridTmp._grid[r]))[c] == NULL)
          {
            empty_pixel_count++;
          }
        }
        else
        {
          empty_pixel_count++;
        }
      }
    }

    // erosion critera
    if(empty_pixel_count >= (size_t)(num_overall_pixel * ratio))
    {
      typename gridElements::iterator temp = it2;
      it2++;
      remove(temp);
    }
    else
      it2++;

    it++;
  }
}

template<class T>
void
SmartGrid<T>::dilate(size_t radius)
{
  //have to save a temporary copy of the depth map
  SmartGrid gridTmp(rows(), cols());

  //first transfer all the existing elements
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    gridTmp.set(it->row(), it->col(), *it);
    it++;
  }

  //now add all the neightbours (within radius)
  it = gridTmp._elements.begin();
  while (it != gridTmp._elements.end())
  {
    for (int r = it->row() - radius; r < it->row() + radius + 1; r++)
    {
      for (int c = it->col() - radius; c < it->col() + radius + 1; c++)
      {
        //check whether this location is inside the image
        if (r >= 0 && r < (int) rows() &&
            c >= 0 && c < (int) cols())
        {
          //check whether that element is missing
          if ((*(_grid[r]))[c] == NULL)
          {
            _elements.push_back(T(r, c));
            (*(_grid[r]))[c] = &const_cast<T &>(_elements.back());
          }
        }
      }
    }

    it++;
  }
}

template<class T>
void
SmartGrid<T>::clean(
  double var_threshold,
  double age_threshold,
  double range_max,
  double range_min)
{
  //first clean the elements and remove any items that are not valid anymore
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    typename gridElements::iterator temp = it;
    it++;

    if (!temp->valid(var_threshold, age_threshold, range_max, range_min))
    {
      (*_grid[temp->row()])[temp->col()] = NULL;
      _elements.erase(temp);
    }
  }
}

template<class T>
void
SmartGrid<T>::clear()
{
  typename gridElements::iterator it = _elements.begin();
  while (it != _elements.end())
  {
    typename gridElements::iterator temp = it;
    it++;
    if (!temp->valid())
    {
      (*_grid[temp->row()])[temp->col()] = NULL;
      _elements.erase(temp);
    }
  }
}

template<class T>
void
SmartGrid<T>::reset()
{
  for (size_t r = 0; r < rows(); r++)
  {
    int size = _grid[r]->size();
    _grid[r]->assign(size, NULL);
  }
  _elements.clear();
}

template<class T>
const T &
SmartGrid<T>::operator()(size_t row, size_t col) const
{
  if ((*_grid[row])[col] != NULL)
    return *((*_grid[row])[col]);
  return _invalid;
}

template<class T>
const T &
SmartGrid<T>::at(size_t row, size_t col) const
{
  if ((*_grid[row])[col] != NULL)
    return *((*_grid[row])[col]);
  return _invalid;
}

template<class T>
size_t
SmartGrid<T>::rows() const
{
  return _grid.size();
}

template<class T>
size_t
SmartGrid<T>::cols() const
{
  return (*_grid.front()).size();
}

template<class T>
void
SmartGrid<T>::set(size_t row, size_t col, const T &value)
{
  if ((*_grid[row])[col] == NULL)
  {
    _elements.push_back(T(row, col));
    (*(_grid[row]))[col] = &const_cast<T &>(_elements.back());
  }
  (*_grid[row])[col]->copy(value);
}

template<class T>
bool
SmartGrid<T>::exists(size_t row, size_t col)
{
  if ((*_grid[row])[col] == NULL)
    return false;
  return true;
}

template<class T>
T &
SmartGrid<T>::get(size_t row, size_t col)
{
  if ((*_grid[row])[col] == NULL)
    return _invalid;
  return *((*_grid[row])[col]);
}

template<class T>
typename SmartGrid<T>::gridElements::iterator
SmartGrid<T>::remove(typename SmartGrid<T>::gridElements::iterator &it)
{
  size_t row = it->row();
  size_t col = it->col();
  (*_grid[row])[col] = NULL;
  return _elements.erase(it);
}

template<class T>
typename SmartGrid<T>::gridElements::iterator
SmartGrid<T>::begin()
{
  return _elements.begin();
}

template<class T>
typename SmartGrid<T>::gridElements::iterator
SmartGrid<T>::end()
{
  return _elements.end();
}

template<class T>
size_t
SmartGrid<T>::size() const
{
  return _elements.size();
}

template<class T>
void
SmartGrid<T>::getNeighbourhood(
  size_t row, size_t col, size_t radius, std::vector<T *> &neighbours)
{
  neighbours.reserve((2 * radius + 1) * (2 * radius + 1));
  for (int r = row - radius; r <= row + radius; r++)
  {
    for (int c = col - radius; c <= col + radius; c++)
    {
      //check whether this location is inside the image
      if (r >= 0 && r < (int) rows() &&
          c >= 0 && c < (int) cols())
      {
        if (exists(r, c) && at(r, c).valid())
          neighbours.push_back(&get(r, c));
      }
    }
  }
}
}
}
#endif //ESVO_CORE_CONTAINER_SMARTGRID_H
