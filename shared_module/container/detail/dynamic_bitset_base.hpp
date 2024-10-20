#pragma once

#include <algorithm>
#include <cassert>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include "../../utils/popcount.hpp"
#include "../../utils/countr_zero.hpp"

namespace detail
{

/**
 * @brief      Dynamic bitset.
 *
 * @details    Data structure used to store a     Vector of bits and apply binary operations to it. The
 *             bits are stored in an optimized way in an underling block type. It is highly inspired
 *             by std\::bitset but with a run-time changeable size.
 *
 *             Preconditions are checked with @a assert but no exception will be thrown if one is
 *             violated (as with std\::bitset).
 *
 * @remark     It is not a Container as it does not provide iterators because of the reference proxy
 *             class used to access the bits.
 *
 * @tparam         Vector         Vector type to use for storing the blocks, must meet the standard
 * @tparam     Block      Block type to use for storing the bits, must be an unsigned integral type
 *
 * @since      1.0.0
 */
template <template <typename> class Vector, typename Block = unsigned long long>
class dynamic_bitset
{
    static_assert(std::is_unsigned<Block>::value, "Block is not an unsigned integral type");

public:
    /**
     * @brief      Same type as @p     Vector<Block>.
     *
     * @since      1.0.0
     */
    typedef Vector<Block> Vector_type;

    /**
     * @brief      Type used to represent the size of a @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    typedef size_t size_type;

    /**
     * @brief      Same type as @p Block.
     *
     * @since      1.0.0
     */
    typedef Block block_type;

    /**
     * @brief      Number of bits that can be stored in a block.
     *
     * @since      1.0.0
     */
    static constexpr size_type bits_per_block = std::numeric_limits<block_type>::digits;

    /**
     * @brief      Maximum value of @ref size_type, returned for invalid positions.
     *
     * @since      1.0.0
     */
    static constexpr size_type npos = std::numeric_limits<size_type>::max();

    /**
     * @brief      Reference to a @ref dynamic_bitset bit.
     *
     * @details    As the bits in the @ref dynamic_bitset class are stored in an optimized way
     *             in blocks, it is not possible for the subscript operators to return a reference
     *             to a boolean. Hence this class is used as a proxy to enable subscript operator of
     *             the @ref dynamic_bitset class to be used as if it was an array of bools.
     *
     * @since      1.0.0
     */
    class reference
    {
    public:
        /**
         * @brief      Constructs a @ref reference to a bit from a @ref dynamic_bitset and a
         *             bit position.
         *
         * @param      bitset   @ref dynamic_bitset containing the bit
         * @param[in]  bit_pos  Position of the bit to reference in the @ref dynamic_bitset
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference(dynamic_bitset<Vector, Block>& bitset, size_type bit_pos);

        /**
         * @brief      Copy constructor.
         *
         * @since      1.0.0
         */
        constexpr reference(const reference&) noexcept = default;

        /**
         * @brief      Move constructor.
         *
         * @since      1.0.0
         */
        constexpr reference(reference&&) noexcept = default;

        /**
         * @brief      Destructor.
         *
         * @since      1.0.0
         */
        ~reference() noexcept = default;

        /**
         * @brief      Assign a value to the referenced bit.
         *
         * @param[in]  v     Value to assign to the referenced bit
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator=(bool v);

        /**
         * @brief      Assign a value to the referenced bit from another @ref reference.
         *
         * @param[in]  rhs   @ref reference to the bit to assign value from
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator=(const reference& rhs);

        /**
         * @brief      Assign a value to the referenced bit from another @ref reference.
         *
         * @param[in]  rhs   @ref reference to the bit to assign value from
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator=(reference&& rhs) noexcept;

        /**
         * @brief      Apply binary operator AND to the referenced bit and a value, and assign the
         *             result to the referenced bit.
         *
         * @param[in]  v     Value to apply binary operator AND with
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator&=(bool v);

        /**
         * @brief      Apply binary operator OR to the referenced bit and a value, and assign the
         *             result to the referenced bit.
         *
         * @param[in]  v     Value to apply binary operator OR with
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator|=(bool v);

        /**
         * @brief      Apply binary operator XOR to the referenced bit and a value, and assign the
         *             result to the referenced bit.
         *
         * @param[in]  v     Value to apply binary operator XOR with
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator^=(bool v);

        /**
         * @brief      Apply binary difference to the referenced bit and a value, and assign the
         *             result to the referenced bit.
         *
         * @details    Equivalent to:
         *             @code
         *             this &= !v;
         *             @endcode
         *
         * @param[in]  v     Value to apply binary difference with
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& operator-=(bool v);

        /**
         * @brief      Return the result of applying unary NOT operator.
         *
         * @return     The opposite of the referenced bit
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        [[nodiscard]] constexpr bool operator~() const;

        /**
         * @brief      bool conversion operator.
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        [[nodiscard]] constexpr operator bool() const;

        /**
         * @brief      Deleted to avoid taking the address of a temporary proxy object.
         *
         * @since      1.0.0
         */
        constexpr void operator&() = delete;

        /**
         * @brief      Set the referenced bit to @a true.
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& set();

        /**
         * @brief      Reset the referenced bit to @a false.
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& reset();

        /**
         * @brief      Flip the referenced bit.
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& flip();

        /**
         * @brief      Assign the value @p v to the referenced bit.
         *
         * @param[in]  v     Value to assign to the bit
         *
         * @return     The @ref reference
         *
         * @complexity Constant.
         *
         * @since      1.0.0
         */
        constexpr reference& assign(bool v);

    private:
        block_type& m_block;
        block_type  m_mask;
    };

    /**
     * @brief      Const reference to a @ref dynamic_bitset bit, type bool.
     *
     * @since      1.0.0
     */
    typedef bool const_reference;

    /**
     * @brief      Copy constructor.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset(const dynamic_bitset<Vector, Block>& other) = default;

    /**
     * @brief      Move constructor.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset(dynamic_bitset<Vector, Block>&& other) noexcept = default;

    /**
     * @brief      Copy assignment operator.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator=(const dynamic_bitset<Vector, Block>& other) = default;

    /**
     * @brief      Move assignment operator.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator=(dynamic_bitset<Vector, Block>&& other) noexcept = default;

    /**
     * @brief      Constructs an empty @ref dynamic_bitset.
     *
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    constexpr explicit dynamic_bitset();

    /**
     * @brief      Constructs a @ref dynamic_bitset of @p nbits bits from an initial value.
     *
     * @details    The first bits are initialized with the bits from @p init_val, if @p nbits \>
     *             std\::numeric_limits\<unsigned long long\>\::digits , all other bits are
     *             initialized to @a false.
     *
     * @param[in]  nbits      Number of bits of the @ref dynamic_bitset
     * @param[in]  init_val   Value to initialize the @ref dynamic_bitset with
     *
     * @complexity Linear in @p nbits / @ref bits_per_block.
     *
     * @since      1.0.0
     */
    constexpr explicit dynamic_bitset(size_type nbits, unsigned long long init_val = 0);

    /**
     * @brief      Constructs a @ref dynamic_bitset using @p init_vals to initialize the first
     *             blocks.
     *
     * @details    The size of the newly created @ref dynamic_bitset is @p init_vals.size() *
     *             @ref bits_per_block.
     *
     * @param[in]  init_vals  Value of the @ref dynamic_bitset first blocks
     *
     * @complexity Linear in @p init_vals.size().
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset(std::initializer_list<block_type> init_vals);

    /**
     * @brief      Constructs a @ref dynamic_bitset from a string or a part of a string.
     *
     * @details    Construct the @ref dynamic_bitset using the characters from @p str in the
     *             range \[@p pos, std\::min(@p pos + @p n, @p str.size())\[.
     *
     * @param[in]  str        String containing the part to use
     * @param[in]  pos        Starting position of the string part to use in @p str
     * @param[in]  n          Number of characters of @p str to use from the starting position
     * @param[in]  zero       Character used to represent @a false bits in @p str
     * @param[in]  one        Character used to represent @a true bits in @p str
     *
     * @tparam     _CharT     Character type of the string
     * @tparam     _Traits    Traits class specifying the operations on the character type of the
     *                        string
     *
     * @pre        @code
     *             pos < str.size()
     *             @endcode
     *
     * @complexity Linear in std\::min(@p n, @p str.size() - @p pos).
     *
     * @since      1.0.0
     */
    template <typename _CharT, typename _Traits>
    constexpr explicit dynamic_bitset(
        std::basic_string_view<_CharT, _Traits>                     str,
        typename std::basic_string_view<_CharT, _Traits>::size_type pos  = 0,
        typename std::basic_string_view<_CharT, _Traits>::size_type n    = std::basic_string_view<_CharT, _Traits>::npos,
        _CharT                                                      zero = _CharT('0'),
        _CharT                                                      one  = _CharT('1'));

    /**
     * @brief      Constructs a @ref dynamic_bitset from a string or a part of a string.
     *
     * @details    Construct the @ref dynamic_bitset using the characters from @p str in the
     *             range \[@p pos, std\::min(@p pos + @p n, @p str.size())\[.
     *
     * @param[in]  str        String containing the part to use
     * @param[in]  pos        Starting position of the string part to use in @p str
     * @param[in]  n          Number of characters of @p str to use from the starting position
     * @param[in]  zero       Character used to represent @a false bits in @p str
     * @param[in]  one        Character used to represent @a true bits in @p str
     *
     * @tparam     _CharT     Character type of the string
     * @tparam     _Traits    Traits class specifying the operations on the character type of the
     *                        string
     *
     * @pre        @code
     *             pos < str.size()
     *             @endcode
     *
     * @complexity Linear in std\::min(@p n, @p str.size() - @p pos).
     *
     * @since      1.0.0
     */
    template <typename _CharT, typename _Traits, typename _Alloc>
    constexpr explicit dynamic_bitset(
        const std::basic_string<_CharT, _Traits, _Alloc>&              str,
        typename std::basic_string<_CharT, _Traits, _Alloc>::size_type pos  = 0,
        typename std::basic_string<_CharT, _Traits, _Alloc>::size_type n    = std::basic_string<_CharT, _Traits, _Alloc>::npos,
        _CharT                                                         zero = _CharT('0'),
        _CharT                                                         one  = _CharT('1'));

    /**
     * @brief      Constructs a @ref dynamic_bitset from a string or a part of a string.
     *
     * @details    Construct the @ref dynamic_bitset using the characters from @p str in the
     *             range \[@p pos, std\::min(@p pos + @p n, @p _Traits\::length(@p str))\[.
     *
     * @param[in]  str        String containing the part to use
     * @param[in]  pos        Starting position of the string part to use
     * @param[in]  n          Number of characters to use from the starting position
     * @param[in]  zero       Character used to represent @a false bits in the string
     * @param[in]  one        Character used to represent 1 @a true bitsn the string
     *
     * @tparam     _CharT     Character type of the string
     * @tparam     _Traits    Traits class specifying the operations on the character type of the
     *                        string
     *
     * @pre        @code
     *             pos < _Traits::length(str)
     *             @endcode
     *
     * @complexity Linear in std\::min(@p n, @p _Traits\::length(@p str) - @p pos).
     *
     * @since      1.0.0
     */
    template <typename _CharT, typename _Traits = std::char_traits<_CharT>>
    constexpr explicit dynamic_bitset(const _CharT*                                 str,
                                      typename std::basic_string<_CharT>::size_type pos  = 0,
                                      typename std::basic_string<_CharT>::size_type n    = std::basic_string<_CharT>::npos,
                                      _CharT                                        zero = _CharT('0'),
                                      _CharT                                        one  = _CharT('1'));

    /**
     * @brief      Destructor.
     *
     * @since      1.0.0
     */
    ~dynamic_bitset() noexcept = default;

    /**
     * @brief      Resize the @ref dynamic_bitset to contain @p nbits bits.
     *
     * @details    Bits keep the value they had before the resize and, if @p nbits is greater than
     *             the current size, new bit are initialized to @p value.
     *
     * @param[in]  nbits  New size of the @ref dynamic_bitset
     * @param[in]  value  Value of the new bits
     *
     * @complexity Linear in the difference between the current size and @p nbits.
     *             Additional complexity possible due to reallocation if capacity is less than @p
     *             nbits.
     *
     * @since      1.0.0
     */
    constexpr void resize(size_type nbits, bool value = false);

    /**
     * @brief      Clears the @ref dynamic_bitset, resize it to 0.
     *
     * @details    Equivalent to:
     *             @code
     *             this.resize(0);
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr void clear();

    /**
     * @brief      Add a new bit with the value @p value at the end of the @ref dynamic_bitset.
     *
     * @details    Increase the size of the bitset by one, the added bit becomes the
     *             most-significant bit.
     *
     * @param[in]  value  Value of the bit to add
     *
     * @complexity Amortized constant.
     *
     * @since      1.0.0
     */
    constexpr void push_back(bool value);

    /**
     * @brief      Remove the last bit of the @ref dynamic_bitset.
     *
     * @details    Decrease the size of the @ref dynamic_bitset by one, does nothing if the
     *             @ref dynamic_bitset is empty.
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    constexpr void pop_back();

    /**
     * @brief      Append a block of bits @p block at the end of the @ref dynamic_bitset.
     *
     * @details    Increase the size of the @ref dynamic_bitset by @ref bits_per_block.
     *
     * @param[in]  block  Block of bits to add
     *
     * @complexity Amortized constant.
     *
     * @since      1.0.0
     */
    constexpr void append(block_type block);

    /**
     * @brief      Append blocks of bits from @p blocks at the end of the @ref dynamic_bitset.
     *
     * @param[in]  blocks  Blocks of bits to add
     *
     * @complexity Linear in the size of @p blocks. Additional complexity possible due
     *             to reallocation if capacity is less than @ref size() + @p blocks.size() * @ref
     *             bits_per_block.
     *
     * @since      1.0.0
     */
    constexpr void append(std::initializer_list<block_type> blocks);

    /**
     * @brief      Append blocks of bits from the range \[@p first, @p last\[ at the end of the @ref
     *             dynamic_bitset.
     *
     * @param[in]  first               First iterator of the range
     * @param[in]  last                Last iterator of the range (after the last element to add)
     *
     * @tparam     BlockInputIterator  Type of the range iterators
     *
     * @complexity Linear in the size of the range. Additional complexity possible due
     *             to reallocation if capacity is less than @ref size() + std\::distance(@p first,
     *             @p last) * @ref bits_per_block.
     *
     * @since      1.0.0
     */
    template <typename BlockInputIterator>
    constexpr void append(BlockInputIterator first, BlockInputIterator last);

    /**
     * @brief      Sets the bits to the result of binary AND on corresponding pairs of bits of *this
     *             and @p rhs.
     *
     * @param[in]  rhs   Right hand side @ref dynamic_bitset of the operator
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre       @code
     *             size() == rhs.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator&=(const dynamic_bitset<Vector, Block>& rhs);

    /**
     * @brief      Sets the bits to the result of binary OR on corresponding pairs of bits of *this
     *             and @p rhs.
     *
     * @param[in]  rhs   Right hand side @ref dynamic_bitset of the operator
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             size() == rhs.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator|=(const dynamic_bitset<Vector, Block>& rhs);

    /**
     * @brief      Sets the bits to the result of binary XOR on corresponding pairs of bits of *this
     *             and @p rhs.
     *
     * @param[in]  rhs   Right hand side @ref dynamic_bitset of the operator
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             size() == rhs.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator^=(const dynamic_bitset<Vector, Block>& rhs);

    /**
     * @brief      Sets the bits to the result of the binary difference between the bits of *this
     *             and @p rhs.
     *
     * @details    Less efficient but equivalent way to get this result:
     *             @code
     *             this &= ~rhs;
     *             @endcode
     *
     * @param[in]  rhs   Right hand side @ref dynamic_bitset of the operator
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             size() == rhs.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator-=(const dynamic_bitset<Vector, Block>& rhs);

    /**
     * @brief      Performs binary shift left of @p shift bits.
     *
     * @details    Zeroes are shifted in, does nothing if @p shift == 0.
     *
     * @param[in]  shift  Number of positions to shift the bits
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator<<=(size_type shift);

    /**
     * @brief      Performs binary shift right of @p shift bits.
     *
     * @details    Zeroes are shifted in, does nothing if @p shift == 0.
     *
     * @param[in]  shift  Number of positions to shift the bits
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& operator>>=(size_type shift);

    /**
     * @brief      Performs binary shift right of @p shift bits.
     *
     * @details    Zeroes are shifted in. Does nothing if @p shift == 0.\n
     *             Equivalent to:
     *             @code
     *             dynamic_bitset<Vector, Block> bitset(*this);
     *             bitset <<= shift;
     *             @endcode
     *
     * @param[in]  shift  Number of positions to shift the bits
     *
     * @return     A new @ref dynamic_bitset containing the shifted bits
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr dynamic_bitset<Vector, Block> operator<<(size_type shift) const;

    /**
     * @brief      Performs binary shift left of @p shift bits.
     *
     * @details    Zeroes are shifted in. Does nothing if @p shift == 0.\n
     *             Equivalent to:
     *             @code
     *             dynamic_bitset<Vector, Block> bitset(*this);
     *             bitset >>= shift;
     *             @endcode
     *
     * @param[in]  shift  Number of positions to shift the bits
     *
     * @return     A new @ref dynamic_bitset containing the shifted bits
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr dynamic_bitset<Vector, Block> operator>>(size_type shift) const;

    /**
     * @brief      Performs a unary NOT on all bits.
     *
     * @details    Equivalent to:
     *             @code
     *             dynamic_bitset<Vector, Block> bitset(*this);
     *             bitset.flip();
     *             @endcode
     *
     * @return     A copy of *this with all bits flipped
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr dynamic_bitset<Vector, Block> operator~() const;

    /**
     * @brief      Set the bits of the range \[@p pos, @p pos + @p len\[ to value @p value.
     *
     * @details    Does nothing if @p len == 0.
     *
     * @param[in]  pos    Position of the first bit of the range
     * @param[in]  len    Length of the range
     * @param[in]  value  Value to set the bits to
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             (pos < size()) && ((len == 0) || (pos + len - 1 < size()))
     *             @endcode
     *
     * @complexity Linear in @p len.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& set(size_type pos, size_type len, bool value);

    /**
     * @brief      Set the bit at the position @p pos to @a true or value @p value.
     *
     * @param[in]  pos    Position of the bit to set
     * @param[in]  value  Value to set the bit to
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& set(size_type pos, bool value = true);

    /**
     * @brief      Set all the bits of the @ref dynamic_bitset to @a true.
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& set();

    /**
     * @brief      Reset the bits of the range \[@p pos, @p pos + @p len\[ to @a false.
     *
     * @param[in]  pos   Position of the first bit of the range
     * @param[in]  len   Length of the range
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             (pos < size()) && ((len == 0) || (pos + len - 1 < size()))
     *             @endcode
     *
     * @complexity Linear in @p len.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& reset(size_type pos, size_type len);

    /**
     * @brief      Reset the bit at the position @p pos to @a false.
     *
     * @param[in]  pos    Position of the bit to reset
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& reset(size_type pos);

    /**
     * @brief      Reset all the bits of the @ref dynamic_bitset to @a false.
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& reset();

    /**
     * @brief      Flip the bits of the range \[@p pos, @p pos + @p len\[.
     *
     * @param[in]  pos   Position of the first bit of the range
     * @param[in]  len   Length of the range
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             (pos < size()) && ((len == 0) || (pos + len - 1 < size()))
     *             @endcode
     *
     * @complexity Linear in @p len.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& flip(size_type pos, size_type len);

    /**
     * @brief      Flip the bit at the position @p pos.
     *
     * @param[in]  pos    Position of the bit to reset
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& flip(size_type pos);

    /**
     * @brief      Flip all the bits of the @ref dynamic_bitset.
     *
     * @return     A reference to the @ref dynamic_bitset *this
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr dynamic_bitset<Vector, Block>& flip();

    /**
     * @brief      Test the value of the bit at position @p pos.
     *
     * @param[in]  pos   Position of the bit to test
     *
     * @return     The tested bit value
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool test(size_type pos) const;

    /**
     * @brief      Test the value of the bit at position @p pos and set it to @a true or value @p
     *             value.
     *
     * @param[in]  pos    Position of the bit to test and set
     * @param[in]  value  Value to set the bit to
     *
     * @return     The tested bit value
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool test_set(size_type pos, bool value = true);

    /**
     * @brief      Checks if all bits are set to @a true.
     *
     * @return     @a true if all bits are set to @a true, otherwise @a false
     *
     * @remark     Return @a true if the @ref dynamic_bitset is empty, the logic is that you
     *             are checking if all bits are set to @a true, meaning none of them is set to @a
     *             false, and in an empty @ref dynamic_bitset no bits are set to @a false.
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool all() const;

    /**
     * @brief      Checks if any bits are set to @a true.
     *
     * @return     @a true if any of the bits is set to @a true, otherwise @a false
     *
     * @remark     Return @a false if the @ref dynamic_bitset is empty, the logic is you are
     *             checking if there is at least one bit set to @a true and in an empty @ref
     *             dynamic_bitset there is no bit set to @a true.
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool any() const;

    /**
     * @brief      Checks if none of the bits are set to @a true.
     *
     * @return     @a true if none of the bits is set to @a true, otherwise @a false
     *
     * @remark     Return @a true if the @ref dynamic_bitset is empty, the logic is that you
     *             are checking if there is no bit set to @a true and in an empty @ref
     *             dynamic_bitset there is no bit that can be set to @a true.
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool none() const;

    /**
     * @brief      Count the number of bits set to @a true.
     *
     * @details    Return 0 if the @ref dynamic_bitset is empty.
     *
     * @return     The number of bits that are set to @a true
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr size_type count() const noexcept;

    /**
     * @brief      Accesses the bit at position @p pos.
     *
     * @param[in]  pos   Position of the bit to access
     *
     * @return     A @ref reference object which allows writing to the requested bit
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr reference operator[](size_type pos);

    /**
     * @brief      Accesses the bit at position @p pos.
     *
     * @param[in]  pos   Position of the bit to access
     *
     * @return     The value of the requested bit
     *
     * @pre        @code
     *             pos < size()
     *             @endcode
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr const_reference operator[](size_type pos) const;

    /**
     * @brief      Give the number of bits of the @ref dynamic_bitset.
     *
     * @return     The number of bits of the @ref dynamic_bitset
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr size_type size() const noexcept;

    /**
     * @brief      Give the number of blocks used by the @ref dynamic_bitset.
     *
     * @return     The number of blocks used by the @ref dynamic_bitset
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr size_type num_blocks() const noexcept;

    /**
     * @brief      Checks if the @ref dynamic_bitset is empty.
     *
     * @details    Equivalent to:
     *             @code
     *             size() == 0;
     *             @endcode
     *
     * @return     @a true if the @ref dynamic_bitset is empty, @a false otherwise
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool empty() const noexcept;

    /**
     * @brief      Give the number of bits that the @ref dynamic_bitset has currently allocated
     *             space for.
     *
     * @return     Capacity of the currently allocated storage.
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr size_type capacity() const noexcept;

    /**
     * @brief      Increase the capacity of the @ref dynamic_bitset to a value that's greater
     *             or equal to @p num_bits.
     *
     * @details    If @p num_bits is greater than the current capacity, new storage is allocated and
     *             all @ref reference on bits of the @ref dynamic_bitset are invalidated,
     *             otherwise the method does nothing.
     *
     * @param[in]  num_bits  New capacity of the @ref dynamic_bitset
     *
     * @complexity At most linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr void reserve(size_type num_bits);

    /**
     * @brief      Requests the removal of unused capacity.
     *
     * @details    It is a non-binding request to reduce the capacity to the size. It depends on the
     *             implementation of std\::vector whether the request is fulfilled.\n If
     *             reallocation occurs, all @ref reference on bits of the @ref dynamic_bitset
     *             are invalidated.
     *
     * @complexity At most linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    constexpr void shrink_to_fit();

    /**
     * @brief      Determines if this @ref dynamic_bitset is a subset of @p bitset.
     *
     * @details    This @ref dynamic_bitset is a subset of @p bitset if, for every bit that is
     *             set in this @ref dynamic_bitset, the corresponding bit in @p bitset a is
     *             also set.\n\n Less efficient but equivalent way to get this result:
     *             @code
     *             res = (this & ~bitset).none();
     *             @endcode
     *
     * @param[in]  bitset  The @ref dynamic_bitset for which to check if this @ref
     *                     dynamic_bitset is a subset
     *
     * @return     @a true if this @ref dynamic_bitset is a subset of @p bitset, @a false
     *             otherwise
     *
     * @remark     The relation "is a subset of" is not symmetric (A being a subset of B doesn't
     *             imply that B is a subset of A) but is antisymmetric (if A is a subset of B and B
     *             is a subset of A, then A == B).
     *
     * @pre        @code
     *             size() == bitset.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool is_subset_of(const dynamic_bitset<Vector, Block>& bitset) const;

    /**
     * @brief      Determines if this @ref dynamic_bitset is a proper subset of @p bitset.
     *
     * @details    This @ref dynamic_bitset is a proper subset of @p bitset if, for every bit
     *             that is set in this @ref dynamic_bitset, the corresponding bit in @p bitset
     *             a is also set and if this @ref dynamic_bitset is different from @p
     *             bitset.\n\n Less efficient but equivalent way to get this result:
     *             @code
     *             res = ((this != bitset) && (this & ~bitset).none());
     *             @endcode
     *
     * @param[in]  bitset  The @ref dynamic_bitset for which to check if this @ref
     *                     dynamic_bitset is a proper subset
     *
     * @return     @a true if this @ref dynamic_bitset is a proper subset of @p bitset, @a
     *             false otherwise
     *
     * @remark     The relation "is a proper subset of" is asymmetric (A being a proper subset of B
     *             imply that B is not a proper subset of A).
     *
     * @pre        @code
     *             size() == bitset.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool is_proper_subset_of(const dynamic_bitset<Vector, Block>& bitset) const;

    /**
     * @brief      Determines if this @ref dynamic_bitset and @p bitset intersect.
     *
     * @details    This @ref dynamic_bitset intersects with @p bitset if for at least one bit
     *             set in this @ref dynamic_bitset, the corresponding bit in @p bitset a is
     *             also set. In other words two bitsets intersect if they have at least one bit set
     *             in common.\n\n Less efficient but equivalent way to get this result:
     *             @code
     *             res = (this & bitset).any();
     *             @endcode
     *
     * @param[in]  bitset  The @ref dynamic_bitset for which to check if this @ref
     *                     dynamic_bitset intersects
     *
     * @return     @a true if this @ref dynamic_bitset intersects with @p bitset, @a false
     *             otherwise
     *
     * @pre        @code
     *             size() == bitset.size()
     *             @endcode
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr bool intersects(const dynamic_bitset<Vector, Block>& bitset) const;

    /**
     * @brief      Find the position of the first bit set in the @ref dynamic_bitset starting
     *             from the least-significant bit.
     *
     * @details    Give the lowest index of the @ref dynamic_bitset with a bit set, or @ref
     *             npos if no bits are set.
     *
     * @return     The position of the first bit set, or @ref npos if no bits are set
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr size_type find_first() const;

    /**
     * @brief      Find the position of the first bit set in the range \[@p prev + 1, @ref size()\[
     *             of the @ref dynamic_bitset starting from the position @p prev + 1.
     *
     * @details    Give the lowest index superior to @p prev of the @ref dynamic_bitset with a
     *             bit set, or @ref npos if no bits are set after the index @p prev.\n If @p prev +
     *             1 \>= @ref size(), return @ref npos.
     *
     * @param[in]  prev  Position of the bit preceding the search range
     *
     * @return     The position of the first bit set after @p prev, or @ref npos if no bits are set
     *             after @p prev
     *
     * @complexity Linear in @ref size() - @p prev.
     *
     * @since      1.0.0
     */
    [[nodiscard]] constexpr size_type find_next(size_type prev) const;

    /**
     * @brief      Exchanges the bits of this @ref dynamic_bitset with those of @p other.
     *
     * @details    All @ref reference on bits of the @ref dynamic_bitset are invalidated.
     *
     * @param      other  @ref dynamic_bitset to exchange bits with
     *
     * @complexity Constant.
     *
     * @since      1.0.0
     */
    constexpr void swap(dynamic_bitset<Vector, Block>& other) noexcept(noexcept(std::swap(m_blocks, other.m_blocks)));

    /**
     * @brief      Generate a string representation of the @ref dynamic_bitset.
     *
     * @details    Uses @p zero to represent bits with value of @a false and @p one to represent
     *             bits with value of @a true. The resulting string contains @ref size() characters
     *             with the first character corresponds to the last (@ref size() - 1th) bit and the
     *             last character corresponding to the first bit.
     *
     * @param[in]  zero     Character to use to represent @a false
     * @param[in]  one      Character to use to represent @a true
     *
     * @tparam     _CharT   Character type of the string
     * @tparam     _Traits  Traits class specifying the operations on the character type of the
     *                      string
     * @tparam     _Alloc   Allocator type used to allocate internal storage of the string
     *
     * @return     The string representing the @ref dynamic_bitset content
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    template <typename _CharT = char, typename _Traits = std::char_traits<_CharT>, typename _Alloc = std::allocator<_CharT>>
    [[nodiscard]] constexpr std::basic_string<_CharT, _Traits, _Alloc> to_string(_CharT zero = _CharT('0'),
                                                                                 _CharT one  = _CharT('1')) const;

    /**
     * @brief      Converts the contents of the bitset to an <tt>unsigned long</tt> integer.
     *
     * @details    The first bit of the bitset corresponds to the least significant digit of the
     *             number and the last bit corresponds to the most significant digit.
     *
     * @return     The numeric value corresponding to the bitset contents.
     *
     * @throws     std::overflow_error  if the value is too large to be represented in an <tt>unsigned
     *                                  long</tt>
     *
     * @complexity Constant.
     *
     * @since      1.3.0
     */
    [[nodiscard]] constexpr unsigned long to_ulong() const;

    /**
     * @brief      Converts the contents of the bitset to an <tt>unsigned long long</tt> integer.
     *
     * @details    The first bit of the bitset corresponds to the least significant digit of the
     *             number and the last bit corresponds to the most significant digit.
     *
     * @return     The numeric value corresponding to the bitset contents.
     *
     * @throws     std::overflow_error  if the value is too large to be represented in an <tt>unsigned long
     *                                  long</tt>
     *
     * @complexity Constant.
     *
     * @since      1.3.0
     */
    [[nodiscard]] constexpr unsigned long long to_ullong() const;

    /**
     * @brief      Iterate on the @ref dynamic_bitset and call @p function with the position of
     *             the bits on.
     *
     * @details    For each set bit, @p function is called as follow:
     *             @code
     *             std::invoke(std::forward<Function>(function), bit_pos, std::forward<Parameters>(parameters)...))
     *             @endcode
     *             where @p bit_pos is the position of the current bit on. Thus @p function
     *             should take a size_t for the current set bit position as first argument, also @p
     *             parameters can be used to pass additional arguments to @p function when it is
     *             called by this method.\n\n @p function can return nothing or a bool, if it return
     *             a bool, the return value indicate if the iteration should continue, @a true to
     *             continue the iteration, @a false to stop, this make it easy to do an early exit.
     *
     * @param      function    Function to call on all bits on, take the current bit position as
     *                         first argument and @p parameters as next arguments
     * @param      parameters  Extra parameters for @p function
     *
     * @tparam     Function    Type of @p function, must take a size_t as first argument and @p
     *                         Parameters as next arguments
     * @tparam     Parameters  Type of @p parameters
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    template <typename Function, typename... Parameters>
    constexpr void iterate_bits_on(Function&& function, Parameters&&... parameters) const;

    /**
     * @brief      Return a pointer to the underlying array serving as blocks storage.
     *
     * @details    The pointer is such that range [@ref data(); @ref data() + @ref num_blocks()) is
     *             always a valid range, even if the container is empty (@ref data() is not
     *             dereferenceable in that case).
     *
     *
     * @post       The bits past the end of the @ref dynamic_bitset in the last block are
     *             guaranteed to be 0s, example:
     *             @code
     *             // random bitset of size 11
     *             std::minstd_rand rand(std::random_device{}());
     *             std::bernoulli_distribution dist;
     *             dynamic_bitset<uint8_t> bitset;
     *             for(size_t i = 0; i < 11; ++i)
     *             {
     *                 bitset.push_back(dist(rand));
     *             }
     *
     *             // the bitset use 2 blocks of 8 bits
     *             // check that unused bits are set to 0
     *             assert(*(bitset.data() + 1) >> 3 == 0);
     *             @endcode
     *
     * @remark     If the @ref dynamic_bitset is empty, this function may or may not return a
     *             null pointer.
     *
     * @return     A pointer to the underlying array serving as blocks storage
     *
     * @complexity Constant.
     *
     * @since      1.2.0
     */
    [[nodiscard]] constexpr block_type* data() noexcept;

    /**
     * @brief      Return a pointer to the underlying array serving as blocks storage.
     *
     * @details    The pointer is such that range [@ref data(); @ref data() + @ref num_blocks()) is
     *             always a valid range, even if the container is empty (@ref data() is not
     *             dereferenceable in that case).
     *
     *
     * @post       The bits past the end of the @ref dynamic_bitset in the last block are
     *             guaranteed to be 0s, example:
     *             @code
     *             // random bitset of size 11
     *             std::minstd_rand rand(std::random_device{}());
     *             std::bernoulli_distribution dist;
     *             dynamic_bitset<uint8_t> bitset;
     *             for(size_t i = 0; i < 11; ++i)
     *             {
     *                 bitset.push_back(dist(rand));
     *             }
     *
     *             // the bitset use 2 blocks of 8 bits
     *             // check that unused bits are set to 0
     *             assert(*(bitset.data() + 1) >> 3 == 0);
     *             @endcode
     *
     * @remark     If the @ref dynamic_bitset is empty, this function may or may not return a
     *             null pointer.
     *
     * @return     A pointer to the underlying array serving as blocks storage
     *
     * @complexity Constant.
     *
     * @since      1.2.0
     */
    [[nodiscard]] constexpr const block_type* data() const noexcept;

    /**
     * @brief      Test if two @ref dynamic_bitset have the same content.
     *
     * @param[in]  lhs         The left hand side @ref dynamic_bitset of the operator
     * @param[in]  rhs         The right hand side @ref dynamic_bitset of the operator
     *
     * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
     * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
     *
     * @return     @a true if they contain the same bits, @a false otherwise
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    template <template <typename> class Vector_, typename Block_>
    friend constexpr bool operator==(const dynamic_bitset<Vector_, Block_>& lhs, const dynamic_bitset<Vector_, Block_>& rhs);

    /**
     * @brief      Test if @p lhs is "less than" @p rhs. The comparison of the two @ref
     *             dynamic_bitset is first on numbers their content represent and then on their
     *             size.
     *
     * @details    The size comparison is necessary for the comparison operators to keep their
     *             properties. For example without the size comparison the "<=" operator (defined
     *             for "A <= B" by "!(B < A)") would no longer be antisymmetric (if A \<= B and B
     *             \<= A, then A == B) because @ref operator==() compare the @ref
     *             dynamic_bitset as a container and not a number. For example with bitsets
     *             A(0011) and B(011), without the size comparison B \< A would be @a false, A \<= B
     *             would be @a true, B \<= A would be @a true, but A == B would be @a false,
     *             breaking the antisymmetric property of the operator. Thus, to respect the
     *             properties of the operators, the size is used as a secondary criteria for the
     *             comparison of @ref dynamic_bitset which content represent the same number.
     *             Therefore, for the previous example with bitsets A(0011) and B(011), B \< A is @a
     *             true, A \<= B is @a false, B \<= A is @a true and A == B is @a false.\n\n If
     *             comparing bitsets @a A and @a B with the content of @a A representing the number
     *             @a a, and the content of @a B representing the number @a b, this operator would
     *             work as follow:
     *             @code
     *             if(a == b)
     *             {
     *                 return A.size() < B.size();
     *             }
     *             else
     *             {
     *                 return a < b;
     *             }
     *             @endcode
     *
     * @remark     The empty @ref dynamic_bitset is the "lowest" of all bitset and for 0-only
     *             bitsets comparison, the shortest is the lowest.
     *
     * @param[in]  lhs         The left hand side @ref dynamic_bitset of the operator
     * @param[in]  rhs         The right hand side @ref dynamic_bitset of the operator
     *
     * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
     * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
     *
     * @return     @a true if @p lhs is "less than" @p rhs
     *
     * @complexity Linear in the size of the @ref dynamic_bitset.
     *
     * @since      1.0.0
     */
    template <template <typename> class Vector_, typename Block_>
    friend constexpr bool operator<(const dynamic_bitset<Vector_, Block_>& lhs, const dynamic_bitset<Vector_, Block_>& rhs);

private:
    template <typename T>
    struct dependent_false : public std::false_type {
    };

    Vector_type m_blocks;
    size_type   m_bits_number;

    static constexpr block_type zero_block           = block_type(0);
    static constexpr block_type one_block            = block_type(~zero_block);
    static constexpr size_type  block_last_bit_index = bits_per_block - 1;

    static constexpr size_type blocks_required(size_type nbits) noexcept;

    static constexpr size_type block_index(size_type pos) noexcept;
    static constexpr size_type bit_index(size_type pos) noexcept;

    static constexpr block_type bit_mask(size_type pos) noexcept;
    static constexpr block_type bit_mask(size_type first, size_type last) noexcept;

    static constexpr void set_block_bits(block_type& block, size_type first, size_type last, bool val = true) noexcept;
    static constexpr void flip_block_bits(block_type& block, size_type first, size_type last) noexcept;

    static constexpr size_type block_count(const block_type& block) noexcept;
    static constexpr size_type block_count(const block_type& block, size_type nbits) noexcept;

    static constexpr size_type count_block_trailing_zero(const block_type& block) noexcept;

    template <typename _CharT, typename _Traits>
    constexpr void init_from_string(std::basic_string_view<_CharT, _Traits>                     str,
                                    typename std::basic_string_view<_CharT, _Traits>::size_type pos,
                                    typename std::basic_string_view<_CharT, _Traits>::size_type n,
                                    _CharT                                                      zero,
                                    _CharT                                                      one);

    constexpr block_type&       get_block(size_type pos);
    constexpr const block_type& get_block(size_type pos) const;
    constexpr block_type&       last_block();
    constexpr block_type        last_block() const;

    // used bits in the last block
    constexpr size_type extra_bits_number() const noexcept;
    // unused bits in the last block
    constexpr size_type unused_bits_number() const noexcept;

    template <typename BinaryOperation>
    constexpr void apply(const dynamic_bitset<Vector, Block>& other, BinaryOperation binary_op);
    template <typename UnaryOperation>
    constexpr void apply(UnaryOperation unary_op);
    constexpr void apply_left_shift(size_type shift);
    constexpr void apply_right_shift(size_type shift);

    // reset unused bits to 0
    constexpr void sanitize();

    // check functions used in asserts
    constexpr bool check_unused_bits() const noexcept;
    constexpr bool check_size() const noexcept;
    constexpr bool check_consistency() const noexcept;
};

//=================================================================================================
// dynamic_bitset external functions declarations
//=================================================================================================

/**
 * @brief      Test if two @ref dynamic_bitset content are different.
 *
 * @details    Defined as:
 *             @code
 *             return !(lhs == rhs);
 *             @endcode
 *             see @ref dynamic_bitset::operator==() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     @a true if they does not contain the same bits, @a false otherwise
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr bool operator!=(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Test if @p lhs is "less than or equal to" @p rhs. The comparison of the two @ref
 *             dynamic_bitset is first on numbers their content represent and then on their size.
 *
 * @details    Defined as:
 *             @code
 *             return !(rhs < lhs);
 *             @endcode
 *             see @ref dynamic_bitset::operator<() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     @a true if @p lhs is "less than or equal to" @p rhs
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr bool operator<=(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Test if @p lhs is "greater than" @p rhs. The comparison of the two @ref
 *             dynamic_bitset is first on numbers their content represent and then on their
 *             size.
 *
 * @details    Defined as:
 *             @code
 *             return rhs < lhs;
 *             @endcode
 *             see @ref dynamic_bitset::operator<() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     @a true if @p lhs is "greater than" @p rhs
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr bool operator>(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Test if @p lhs is "greater than or equal to" @p rhs. The comparison of the two @ref
 *             dynamic_bitset is first on numbers their content represent and then on their
 *             size.
 *
 * @details    Defined as:
 *             @code
 *             return !(lhs < rhs);
 *             @endcode
 *             see @ref dynamic_bitset::operator<() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     @a true if @p lhs is "greater than or equal to" @p rhs
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr bool operator>=(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Performs binary AND on corresponding pairs of bits of @p lhs and @p rhs.
 *
 * @details    Defined as:
 *             @code
 *             dynamic_bitset<Vector, Block> result(lhs);
 *             return result &= rhs;
 *             @endcode
 *             see @ref dynamic_bitset::operator&=() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     A @ref dynamic_bitset with each bit being the result of a binary AND between the
 *             corresponding pair of bits of @p lhs and @p rhs
 *
 * @pre        @code
 *             lhs.size() == rhs.size()
 *             @endcode
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator&(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Performs binary OR on corresponding pairs of bits of @p lhs and @p rhs.
 *
 * @details    Defined as:
 *             @code
 *             dynamic_bitset<Vector, Block> result(lhs);
 *             return result |= rhs;
 *             @endcode
 *             see @ref dynamic_bitset::operator|=() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     A @ref dynamic_bitset with each bit being the result of a binary OR between the
 *             corresponding pair of bits of @p lhs and @p rhs
 *
 * @pre        @code
 *             lhs.size() == rhs.size()
 *             @endcode
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator|(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Performs binary XOR on corresponding pairs of bits of @p lhs and @p rhs.
 *
 * @details    Defined as:
 *             @code
 *             dynamic_bitset<Vector, Block> result(lhs);
 *             return result ^= rhs;
 *             @endcode
 *             see @ref dynamic_bitset::operator^=() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     A @ref dynamic_bitset with each bit being the result of a binary XOR between the
 *             corresponding pair of bits of @p lhs and @p rhs
 *
 * @pre        @code
 *             lhs.size() == rhs.size()
 *             @endcode
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator^(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Performs binary difference between bits of @p lhs and @p rhs.
 *
 * @details    Defined as:
 *             @code
 *             dynamic_bitset<Vector, Block> result(lhs);
 *             return result -= rhs;
 *             @endcode
 *             see @ref dynamic_bitset::operator-=() for more informations.
 *
 * @param[in]  lhs        The left hand side @ref dynamic_bitset of the operator
 * @param[in]  rhs        The right hand side @ref dynamic_bitset of the operator
 *
 * @tparam     Vector_          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block_      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     A @ref dynamic_bitset with each bit being the result of the binary difference
 *             between the corresponding bits of @p lhs and @p rhs
 *
 * @pre        @code
 *             lhs.size() == rhs.size()
 *             @endcode
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator-(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs);

/**
 * @brief      Insert a string representation of this @ref dynamic_bitset to a character
 *             stream.
 *
 * @details    The string representation written is the same as if generated with @ref
 *             dynamic_bitset::to_string() with default parameter, using '1' for @a true bits
 *             and '0' for @a false bits.
 *
 * @param      os         Character stream to write to
 * @param[in]  bitset     @ref dynamic_bitset to write
 *
 * @tparam     _CharT     Character type of the character stream
 * @tparam     _Traits    Traits class specifying the operations on the character type of the
 *                        character stream
 * @tparam     Vector          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     @p os
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <typename _CharT, typename _Traits, typename Block, template <typename> class Vector>
constexpr std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& os,
                                                          const dynamic_bitset<Vector, Block>& bitset);

/**
 * @brief      Extract a @ref dynamic_bitset from a character stream using its string
 *             representation.
 *
 * @details    The string representation expected is the same as if generated with @ref
 *             dynamic_bitset::to_string() with default parameter, using '1' for @a true bits
 *             and '0' for @a false bits. On success the content of @p bitset is cleared before
 *             writing to it. The extraction starts by skipping leading whitespace then take the
 *             characters one by one and stop if @p is.good() return @a false or the next character
 *             is neither _CharT('0') nor _CharT('1').
 *
 * @param      is         Character stream to read from
 * @param      bitset     @ref dynamic_bitset to write to
 *
 * @tparam     _CharT     Character type of the character stream
 * @tparam     _Traits    Traits class specifying the operations on the character type of the
 *                        character stream
 * @tparam     Vector          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @return     @p is
 *
 * @complexity Linear in the size of the @ref dynamic_bitset.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <typename _CharT, typename _Traits, typename Block, template <typename> class Vector>
constexpr std::basic_istream<_CharT, _Traits>& operator>>(std::basic_istream<_CharT, _Traits>& is,
                                                          dynamic_bitset<Vector, Block>&       bitset);

/**
 * @brief      Exchange the content of @p bitset1 and @p bitset2.
 *
 * @details    Defined as:
 *             @code
 *             bitset1.swap(bitset2);
 *             @endcode
 *             see @ref dynamic_bitset::swap() for more informations.
 *
 * @param      bitset1    @ref dynamic_bitset to be swapped
 * @param      bitset2    @ref dynamic_bitset to be swapped
 *
 * @tparam     Vector          Vector type used by @p lhs and @p rhs for storing the bits
 * @tparam     Block      Block type used by @p lhs and @p rhs for storing the bits
 *
 * @complexity Constant.
 *
 * @since      1.0.0
 *
 * @relatesalso dynamic_bitset
 */
template <template <typename> class Vector, typename Block>
constexpr void swap(dynamic_bitset<Vector, Block>& bitset1,
                    dynamic_bitset<Vector, Block>& bitset2) noexcept(noexcept(bitset1.swap(bitset2)));

//=================================================================================================
// dynamic_bitset::reference functions implementations
//=================================================================================================

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>::reference::reference(dynamic_bitset<Vector, Block>& bitset, size_type bit_pos)
    : m_block(bitset.get_block(bit_pos)), m_mask(dynamic_bitset<Vector, Block>::bit_mask(bit_pos))
{
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator=(bool v)
{
    assign(v);
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator=(
    const dynamic_bitset<Vector, Block>::reference& rhs)
{
    assign(rhs);
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator=(
    dynamic_bitset::reference&& rhs) noexcept
{
    assign(rhs);
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator&=(bool v)
{
    if (!v) { reset(); }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator|=(bool v)
{
    if (v) { set(); }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator^=(bool v)
{
    if (v) { flip(); }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::operator-=(bool v)
{
    if (v) { reset(); }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::reference::operator~() const
{
    return (m_block & m_mask) == zero_block;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>::reference::operator bool() const
{
    return (m_block & m_mask) != zero_block;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::set()
{
    m_block |= m_mask;
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::reset()
{
    m_block &= static_cast<block_type>(~m_mask);
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::flip()
{
    m_block ^= m_mask;
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference& dynamic_bitset<Vector, Block>::reference::assign(bool v)
{
    if (v) {
        set();
    } else {
        reset();
    }
    return *this;
}

//=================================================================================================
// dynamic_bitset public functions implementations
//=================================================================================================

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>::dynamic_bitset() : m_blocks(), m_bits_number(0)
{
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>::dynamic_bitset(size_type nbits, unsigned long long init_val)
    : m_blocks(blocks_required(nbits)), m_bits_number(nbits)
{
    if (nbits == 0 || init_val == 0) { return; }

    constexpr size_t ull_bits_number          = std::numeric_limits<unsigned long long>::digits;
    constexpr size_t init_val_required_blocks = ull_bits_number / bits_per_block;
    if constexpr (init_val_required_blocks == 1) {
        m_blocks[0] = init_val;
    } else {
        const unsigned long long block_mask     = static_cast<unsigned long long>(one_block);
        const size_t             blocks_to_init = std::min(m_blocks.size(), init_val_required_blocks);
        for (size_t i = 0; i < blocks_to_init; ++i) {
            m_blocks[i] = block_type((init_val >> (i * bits_per_block) & block_mask));
        }
    }
    sanitize();
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>::dynamic_bitset(std::initializer_list<block_type> init_vals)
    : m_blocks(), m_bits_number(0)
{
    append(init_vals);
}

template <template <typename> class Vector, typename Block>
template <typename _CharT, typename _Traits>
constexpr dynamic_bitset<Vector, Block>::dynamic_bitset(std::basic_string_view<_CharT, _Traits>                     str,
                                                        typename std::basic_string_view<_CharT, _Traits>::size_type pos,
                                                        typename std::basic_string_view<_CharT, _Traits>::size_type n,
                                                        _CharT                                                      zero,
                                                        _CharT                                                      one)
    : m_blocks(), m_bits_number(0)
{
    assert(pos < str.size());
    init_from_string(str, pos, n, zero, one);
}

template <template <typename> class Vector, typename Block>
template <typename _CharT, typename _Traits, typename _Alloc>
constexpr dynamic_bitset<Vector, Block>::dynamic_bitset(const std::basic_string<_CharT, _Traits, _Alloc>&              str,
                                                        typename std::basic_string<_CharT, _Traits, _Alloc>::size_type pos,
                                                        typename std::basic_string<_CharT, _Traits, _Alloc>::size_type n,
                                                        _CharT                                                         zero,
                                                        _CharT                                                         one)
    : m_blocks(), m_bits_number(0)
{
    assert(pos < str.size());
    init_from_string(std::basic_string_view<_CharT, _Traits>(str), pos, n, zero, one);
}

template <template <typename> class Vector, typename Block>
template <typename _CharT, typename _Traits>
constexpr dynamic_bitset<Vector, Block>::dynamic_bitset(const _CharT*                                 str,
                                                        typename std::basic_string<_CharT>::size_type pos,
                                                        typename std::basic_string<_CharT>::size_type n,
                                                        _CharT                                        zero,
                                                        _CharT                                        one)
    : m_blocks(), m_bits_number(0)
{
    init_from_string(std::basic_string_view<_CharT, _Traits>(str), pos, n, zero, one);
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::resize(size_type nbits, bool value)
{
    if (nbits == m_bits_number) { return; }

    const size_type old_num_blocks = num_blocks();
    const size_type new_num_blocks = blocks_required(nbits);

    const block_type init_value = value ? one_block : zero_block;
    if (new_num_blocks != old_num_blocks) { m_blocks.resize(new_num_blocks, init_value); }

    if (value && nbits > m_bits_number && old_num_blocks > 0) {
        // set value of the new bits in the old last block
        const size_type extra_bits = extra_bits_number();
        if (extra_bits > 0) { m_blocks[old_num_blocks - 1] |= static_cast<block_type>(init_value << extra_bits); }
    }

    m_bits_number = nbits;
    sanitize();
    assert(check_consistency());
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::clear()
{
    m_blocks.clear();
    m_bits_number = 0;
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::push_back(bool value)
{
    const size_type new_last_bit = m_bits_number++;
    if (m_bits_number <= m_blocks.size() * bits_per_block) {
        if (value) { set(new_last_bit, value); }
    } else {
        m_blocks.push_back(block_type(value));
    }
    assert(operator[](new_last_bit) == value);
    assert(check_consistency());
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::pop_back()
{
    if (empty()) { return; }

    --m_bits_number;
    if (m_blocks.size() > blocks_required(m_bits_number)) {
        m_blocks.pop_back();
        // no extra bits: sanitize not required
        assert(extra_bits_number() == 0);
    } else {
        sanitize();
    }
    assert(check_consistency());
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::append(block_type block)
{
    const size_type extra_bits = extra_bits_number();
    if (extra_bits == 0) {
        m_blocks.push_back(block);
    } else {
        last_block() |= static_cast<block_type>(block << extra_bits);
        m_blocks.push_back(block_type(block >> (bits_per_block - extra_bits)));
    }

    m_bits_number += bits_per_block;
    assert(check_consistency());
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::append(std::initializer_list<block_type> blocks)
{
    if (blocks.size() == 0) { return; }

    append(std::cbegin(blocks), std::cend(blocks));
}

template <template <typename> class Vector, typename Block>
template <typename BlockInputIterator>
constexpr void dynamic_bitset<Vector, Block>::append(BlockInputIterator first, BlockInputIterator last)
{
    if (first == last) { return; }

    // if random access iterators, std::distance complexity is constant
    if constexpr (std::is_same_v<typename std::iterator_traits<BlockInputIterator>::iterator_category,
                                 std::random_access_iterator_tag>) {
        assert(std::distance(first, last) > 0);
        m_blocks.reserve(m_blocks.size() + static_cast<size_type>(std::distance(first, last)));
    }

    const size_type extra_bits  = extra_bits_number();
    const size_type unused_bits = unused_bits_number();
    if (extra_bits == 0) {
        auto pos = m_blocks.insert(std::end(m_blocks), first, last);
        assert(std::distance(pos, std::end(m_blocks)) > 0);
        m_bits_number += static_cast<size_type>(std::distance(pos, std::end(m_blocks))) * bits_per_block;
    } else {
        last_block()     |= static_cast<block_type>(*first << extra_bits);
        block_type block  = block_type(*first >> unused_bits);
        ++first;
        while (first != last) {
            block |= static_cast<block_type>(*first << extra_bits);
            m_blocks.push_back(block);
            m_bits_number += bits_per_block;
            block          = block_type(*first >> unused_bits);
            ++first;
        }
        m_blocks.push_back(block);
        m_bits_number += bits_per_block;
    }

    assert(check_consistency());
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::operator&=(const dynamic_bitset<Vector, Block>& rhs)
{
    assert(size() == rhs.size());
    // apply(rhs, std::bit_and());
    for (size_type i = 0; i < m_blocks.size(); ++i) { m_blocks[i] &= rhs.m_blocks[i]; }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::operator|=(const dynamic_bitset<Vector, Block>& rhs)
{
    assert(size() == rhs.size());
    // apply(rhs, std::bit_or());
    for (size_type i = 0; i < m_blocks.size(); ++i) { m_blocks[i] |= rhs.m_blocks[i]; }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::operator^=(const dynamic_bitset<Vector, Block>& rhs)
{
    assert(size() == rhs.size());
    // apply(rhs, std::bit_xor());
    for (size_type i = 0; i < m_blocks.size(); ++i) { m_blocks[i] ^= rhs.m_blocks[i]; }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::operator-=(const dynamic_bitset<Vector, Block>& rhs)
{
    assert(size() == rhs.size());
    // apply(rhs, [](const block_type& x, const block_type& y) { return (x & ~y); });
    for (size_type i = 0; i < m_blocks.size(); ++i) { m_blocks[i] &= static_cast<block_type>(~rhs.m_blocks[i]); }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::operator<<=(size_type shift)
{
    if (shift != 0) {
        if (shift >= m_bits_number) {
            reset();
        } else {
            apply_left_shift(shift);
            sanitize(); // unused bits can have changed, reset them to 0
        }
    }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::operator>>=(size_type shift)
{
    if (shift != 0) {
        if (shift >= m_bits_number) {
            reset();
        } else {
            apply_right_shift(shift);
        }
    }
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> dynamic_bitset<Vector, Block>::operator<<(size_type shift) const
{
    return dynamic_bitset<Vector, Block>(*this) <<= shift;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> dynamic_bitset<Vector, Block>::operator>>(size_type shift) const
{
    return dynamic_bitset<Vector, Block>(*this) >>= shift;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> dynamic_bitset<Vector, Block>::operator~() const
{
    dynamic_bitset<Vector, Block> bitset(*this);
    bitset.flip();
    return bitset;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::set(size_type pos, size_type len, bool value)
{
    assert(pos < size());
    if (len == 0) { return *this; }
    assert(pos + len - 1 < size());

    const size_type first_block     = block_index(pos);
    const size_type last_block      = block_index(pos + len - 1);
    const size_type first_bit_index = bit_index(pos);
    const size_type last_bit_index  = bit_index(pos + len - 1);

    if (first_block == last_block) {
        set_block_bits(m_blocks[first_block], first_bit_index, last_bit_index, value);
    } else {
        size_type first_full_block = first_block;
        size_type last_full_block  = last_block;

        if (first_bit_index != 0) {
            ++first_full_block; // first block is not full
            set_block_bits(m_blocks[first_block], first_bit_index, block_last_bit_index, value);
        }

        if (last_bit_index != block_last_bit_index) {
            --last_full_block; // last block is not full
            set_block_bits(m_blocks[last_block], 0, last_bit_index, value);
        }

        const block_type full_block = value ? one_block : zero_block;
        for (size_type i = first_full_block; i <= last_full_block; ++i) { m_blocks[i] = full_block; }
    }

    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::set(size_type pos, bool value)
{
    assert(pos < size());

    if (value) {
        m_blocks[block_index(pos)] |= bit_mask(pos);
    } else {
        m_blocks[block_index(pos)] &= static_cast<block_type>(~bit_mask(pos));
    }

    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::set()
{
    std::fill(std::begin(m_blocks), std::end(m_blocks), one_block);
    sanitize();
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::reset(size_type pos, size_type len)
{
    return set(pos, len, false);
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::reset(size_type pos)
{
    return set(pos, false);
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::reset()
{
    std::fill(std::begin(m_blocks), std::end(m_blocks), zero_block);
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::flip(size_type pos, size_type len)
{
    assert(pos < size());
    if (len == 0) { return *this; }
    assert(pos + len - 1 < size());

    const size_type first_block     = block_index(pos);
    const size_type last_block      = block_index(pos + len - 1);
    const size_type first_bit_index = bit_index(pos);
    const size_type last_bit_index  = bit_index(pos + len - 1);

    if (first_block == last_block) {
        flip_block_bits(m_blocks[first_block], first_bit_index, last_bit_index);
    } else {
        size_type first_full_block = first_block;
        size_type last_full_block  = last_block;

        if (first_bit_index != 0) {
            ++first_full_block; // first block is not full
            flip_block_bits(m_blocks[first_block], first_bit_index, block_last_bit_index);
        }

        if (last_bit_index != block_last_bit_index) {
            --last_full_block; // last block is not full
            flip_block_bits(m_blocks[last_block], 0, last_bit_index);
        }

        for (size_type i = first_full_block; i <= last_full_block; ++i) { m_blocks[i] = block_type(~m_blocks[i]); }
    }

    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::flip(size_type pos)
{
    assert(pos < size());
    m_blocks[block_index(pos)] ^= bit_mask(pos);
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block>& dynamic_bitset<Vector, Block>::flip()
{
    std::transform(std::cbegin(m_blocks), std::cend(m_blocks), std::begin(m_blocks), std::bit_not<block_type>());
    sanitize();
    return *this;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::test(size_type pos) const
{
    assert(pos < size());
    return (m_blocks[block_index(pos)] & bit_mask(pos)) != zero_block;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::test_set(size_type pos, bool value)
{
    bool const result = test(pos);
    if (result != value) { set(pos, value); }
    return result;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::all() const
{
    if (empty()) { return true; }

    const block_type full_block = one_block;
    if (extra_bits_number() == 0) {
        for (const block_type& block : m_blocks) {
            if (block != full_block) { return false; }
        }
    } else {
        for (size_type i = 0; i < m_blocks.size() - 1; ++i) {
            if (m_blocks[i] != full_block) { return false; }
        }
        if (last_block() != (full_block >> unused_bits_number())) { return false; }
    }
    return true;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::any() const
{
    for (const block_type& block : m_blocks) {
        if (block != zero_block) { return true; }
    }
    return false;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::none() const
{
    return !any();
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::count() const noexcept
{
    if (empty()) { return 0; }

    const size_type count = static_cast<size_type>(popcnt(m_blocks.data(), m_blocks.size() * sizeof(block_type)));
    return count;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::reference dynamic_bitset<Vector, Block>::operator[](size_type pos)
{
    assert(pos < size());
    return dynamic_bitset<Vector, Block>::reference(*this, pos);
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::const_reference dynamic_bitset<Vector, Block>::operator[](size_type pos) const
{
    return test(pos);
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::size() const noexcept
{
    return m_bits_number;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::num_blocks() const noexcept
{
    return m_blocks.size();
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::empty() const noexcept
{
    return size() == 0;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::capacity() const noexcept
{
    return m_blocks.capacity() * bits_per_block;
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::reserve(size_type num_bits)
{
    m_blocks.reserve(blocks_required(num_bits));
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::shrink_to_fit()
{
    m_blocks.shrink_to_fit();
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::is_subset_of(const dynamic_bitset<Vector, Block>& bitset) const
{
    assert(size() == bitset.size());
    for (size_type i = 0; i < m_blocks.size(); ++i) {
        if ((m_blocks[i] & ~bitset.m_blocks[i]) != zero_block) { return false; }
    }
    return true;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::is_proper_subset_of(const dynamic_bitset<Vector, Block>& bitset) const
{
    assert(size() == bitset.size());
    bool is_proper = false;
    for (size_type i = 0; i < m_blocks.size(); ++i) {
        const block_type& self_block  = m_blocks[i];
        const block_type& other_block = bitset.m_blocks[i];

        if ((self_block & ~other_block) != zero_block) { return false; }
        if ((~self_block & other_block) != zero_block) { is_proper = true; }
    }
    return is_proper;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::intersects(const dynamic_bitset<Vector, Block>& bitset) const
{
    const size_type min_blocks_number = std::min(m_blocks.size(), bitset.m_blocks.size());
    for (size_type i = 0; i < min_blocks_number; ++i) {
        if ((m_blocks[i] & bitset.m_blocks[i]) != zero_block) { return true; }
    }
    return false;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::find_first() const
{
    for (size_type i = 0; i < m_blocks.size(); ++i) {
        if (m_blocks[i] != zero_block) { return i * bits_per_block + count_block_trailing_zero(m_blocks[i]); }
    }
    return npos;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::find_next(size_type prev) const
{
    if (empty() || prev >= (size() - 1)) { return npos; }

    const size_type  first_bit           = prev + 1;
    const size_type  first_block         = block_index(first_bit);
    const size_type  first_bit_index     = bit_index(first_bit);
    const block_type first_block_shifted = block_type(m_blocks[first_block] >> first_bit_index);

    if (first_block_shifted != zero_block) {
        return first_bit + count_block_trailing_zero(first_block_shifted);
    } else {
        for (size_type i = first_block + 1; i < m_blocks.size(); ++i) {
            if (m_blocks[i] != zero_block) { return i * bits_per_block + count_block_trailing_zero(m_blocks[i]); }
        }
    }
    return npos;
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::swap(dynamic_bitset<Vector, Block>& other) noexcept(
    noexcept(std::swap(m_blocks, other.m_blocks)))
{
    std::swap(m_blocks, other.m_blocks);
    std::swap(m_bits_number, other.m_bits_number);
}

template <template <typename> class Vector, typename Block>
template <typename _CharT, typename _Traits, typename _Alloc>
constexpr std::basic_string<_CharT, _Traits, _Alloc> dynamic_bitset<Vector, Block>::to_string(_CharT zero, _CharT one) const
{
    const size_type                            len = size();
    std::basic_string<_CharT, _Traits, _Alloc> str(len, zero);
    for (size_type i_block = 0; i_block < m_blocks.size(); ++i_block) {
        if (m_blocks[i_block] == zero_block) { continue; }
        block_type      mask  = block_type(1);
        const size_type limit = i_block * bits_per_block < len ? len - i_block * bits_per_block : bits_per_block;
        for (size_type i_bit = 0; i_bit < limit; ++i_bit) {
            if ((m_blocks[i_block] & mask) != zero_block) {
                _Traits::assign(str[len - (i_block * bits_per_block + i_bit + 1)], one);
            }
            // mask <<= 1; not used because it trigger -Wconversion
            // because of integral promotion for block_type smaller than int
            mask = static_cast<block_type>(mask << 1);
        }
    }
    return str;
}

template <template <typename> class Vector, typename Block>
constexpr unsigned long dynamic_bitset<Vector, Block>::to_ulong() const
{
    if (m_bits_number == 0) { return 0; }

    constexpr size_t ul_bits_number = std::numeric_limits<unsigned long>::digits;
    if (find_next(ul_bits_number - 1) != npos) { throw std::overflow_error("dynamic_bitset::to_ulong"); }

    unsigned long   result             = 0;
    const size_type result_bits_number = std::min(ul_bits_number, m_bits_number);
    for (size_type i_block = 0; i_block <= block_index(result_bits_number - 1); ++i_block) {
        result |= (static_cast<unsigned long>(m_blocks[i_block]) << (i_block * bits_per_block));
    }

    return result;
}

template <template <typename> class Vector, typename Block>
constexpr unsigned long long dynamic_bitset<Vector, Block>::to_ullong() const
{
    if (m_bits_number == 0) { return 0; }

    constexpr size_t ull_bits_number = std::numeric_limits<unsigned long long>::digits;
    if (find_next(ull_bits_number - 1) != npos) { throw std::overflow_error("dynamic_bitset::to_ullong"); }

    unsigned long long result             = 0;
    const size_type    result_bits_number = std::min(ull_bits_number, m_bits_number);
    for (size_type i_block = 0; i_block <= block_index(result_bits_number - 1); ++i_block) {
        result |= (static_cast<unsigned long long>(m_blocks[i_block]) << (i_block * bits_per_block));
    }

    return result;
}

template <template <typename> class Vector, typename Block>
template <typename Function, typename... Parameters>
constexpr void dynamic_bitset<Vector, Block>::iterate_bits_on(Function&& function, Parameters&&... parameters) const
{
    if constexpr (!std::is_invocable_v<Function, size_t, Parameters...>) {
        static_assert(dependent_false<Function>::value, "Function take invalid arguments");
        // function should take (size_t, parameters...) as arguments
    }

    if constexpr (std::is_same_v<std::invoke_result_t<Function, size_t, Parameters...>, void>) {
        size_type i_bit = find_first();
        while (i_bit != npos) {
            std::invoke(std::forward<Function>(function), i_bit, std::forward<Parameters>(parameters)...);
            i_bit = find_next(i_bit);
        }
    } else if constexpr (std::is_convertible_v<std::invoke_result_t<Function, size_t, Parameters...>, bool>) {
        size_type i_bit = find_first();
        while (i_bit != npos) {
            if (!std::invoke(std::forward<Function>(function), i_bit, std::forward<Parameters>(parameters)...)) { break; }
            i_bit = find_next(i_bit);
        }
    } else {
        static_assert(dependent_false<Function>::value, "Function have invalid return type");
        // return type should be void, or convertible to bool
    }
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::block_type* dynamic_bitset<Vector, Block>::data() noexcept
{
    return m_blocks.data();
}

template <template <typename> class Vector, typename Block>
constexpr const typename dynamic_bitset<Vector, Block>::block_type* dynamic_bitset<Vector, Block>::data() const noexcept
{
    return m_blocks.data();
}

template <template <typename> class Vector_, typename Block_>
[[nodiscard]] constexpr bool operator==(const dynamic_bitset<Vector_, Block_>& lhs, const dynamic_bitset<Vector_, Block_>& rhs)
{
    return (lhs.m_bits_number == rhs.m_bits_number) && (lhs.m_blocks == rhs.m_blocks);
}

template <template <typename> class Vector_, typename Block_>
[[nodiscard]] constexpr bool operator<(const dynamic_bitset<Vector_, Block_>& lhs, const dynamic_bitset<Vector_, Block_>& rhs)
{
    using size_type                 = typename dynamic_bitset<Vector_, Block_>::size_type;
    using block_type                = typename dynamic_bitset<Vector_, Block_>::block_type;
    const size_type lhs_size        = lhs.size();
    const size_type rhs_size        = rhs.size();
    const size_type lhs_blocks_size = lhs.m_blocks.size();
    const size_type rhs_blocks_size = rhs.m_blocks.size();

    if (lhs_size == rhs_size) {
        // if comparison of two empty bitsets
        if (lhs_size == 0) { return false; }

        for (size_type i = lhs_blocks_size - 1; i > 0; --i) {
            if (lhs.m_blocks[i] != rhs.m_blocks[i]) { return lhs.m_blocks[i] < rhs.m_blocks[i]; }
        }
        return lhs.m_blocks[0] < rhs.m_blocks[0];
    }

    // empty bitset inferior to 0-only bitset
    if (lhs_size == 0) { return true; }
    if (rhs_size == 0) { return false; }

    const bool                             rhs_longer           = rhs_size > lhs_size;
    const dynamic_bitset<Vector_, Block_>& longest_bitset       = rhs_longer ? rhs : lhs;
    const size_type                        longest_blocks_size  = std::max(lhs_blocks_size, rhs_blocks_size);
    const size_type                        shortest_blocks_size = std::min(lhs_blocks_size, rhs_blocks_size);
    for (size_type i = longest_blocks_size - 1; i >= shortest_blocks_size; --i) {
        if (longest_bitset.m_blocks[i] != block_type(0)) { return rhs_longer; }
    }

    for (size_type i = shortest_blocks_size - 1; i > 0; --i) {
        if (lhs.m_blocks[i] != rhs.m_blocks[i]) { return lhs.m_blocks[i] < rhs.m_blocks[i]; }
    }
    if (lhs.m_blocks[0] != rhs.m_blocks[0]) { return lhs.m_blocks[0] < rhs.m_blocks[0]; }
    return lhs_size < rhs_size;
}

//=================================================================================================
// dynamic_bitset private functions implementations
//=================================================================================================

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::blocks_required(
    size_type nbits) noexcept
{
    return nbits / bits_per_block + static_cast<size_type>(nbits % bits_per_block > 0);
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::block_index(size_type pos) noexcept
{
    return pos / bits_per_block;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::bit_index(size_type pos) noexcept
{
    return pos % bits_per_block;
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::block_type dynamic_bitset<Vector, Block>::bit_mask(size_type pos) noexcept
{
    return block_type(block_type(1) << bit_index(pos));
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::block_type dynamic_bitset<Vector, Block>::bit_mask(size_type first,
                                                                                                     size_type last) noexcept
{
    first = bit_index(first);
    last  = bit_index(last);
    if (last == (block_last_bit_index)) {
        return block_type(one_block << first);
    } else {
        return block_type(((block_type(1) << (last + 1)) - 1) ^ ((block_type(1) << first) - 1));
    }
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::set_block_bits(block_type& block,
                                                             size_type   first,
                                                             size_type   last,
                                                             bool        val) noexcept
{
    if (val) {
        block |= bit_mask(first, last);
    } else {
        block &= static_cast<block_type>(~bit_mask(first, last));
    }
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::flip_block_bits(block_type& block, size_type first, size_type last) noexcept
{
    block ^= bit_mask(first, last);
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::block_count(
    const block_type& block) noexcept
{
    return static_cast<size_type>(popcnt(&block, sizeof(block_type)));
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::block_count(const block_type& block,
                                                                                                       size_type nbits) noexcept
{
    assert(nbits <= bits_per_block);
    const block_type shifted_block = block_type(block << (bits_per_block - nbits));
    return static_cast<size_type>(popcnt(&shifted_block, sizeof(block_type)));
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::count_block_trailing_zero(
    const block_type& block) noexcept
{
    assert(block != zero_block);
    return static_cast<size_type>(__countr_zero(block));
}

template <template <typename> class Vector, typename Block>
template <typename _CharT, typename _Traits>
constexpr void dynamic_bitset<Vector, Block>::init_from_string(std::basic_string_view<_CharT, _Traits>                     str,
                                                               typename std::basic_string_view<_CharT, _Traits>::size_type pos,
                                                               typename std::basic_string_view<_CharT, _Traits>::size_type n,
                                                               [[maybe_unused]] _CharT                                     zero,
                                                               _CharT                                                      one)
{
    assert(pos < str.size());

    const size_type size = std::min(n, str.size() - pos);
    m_bits_number        = size;

    m_blocks.clear();
    m_blocks.resize(blocks_required(size));
    for (size_t i = 0; i < size; ++i) {
        const _CharT c = str[(pos + size - 1) - i];
        assert(c == zero || c == one);
        if (c == one) { set(i); }
    }
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::block_type& dynamic_bitset<Vector, Block>::get_block(size_type pos)
{
    return m_blocks[block_index(pos)];
}

template <template <typename> class Vector, typename Block>
constexpr const typename dynamic_bitset<Vector, Block>::block_type& dynamic_bitset<Vector, Block>::get_block(
    size_type pos) const
{
    return m_blocks[block_index(pos)];
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::block_type& dynamic_bitset<Vector, Block>::last_block()
{
    return m_blocks[m_blocks.size() - 1];
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::block_type dynamic_bitset<Vector, Block>::last_block() const
{
    return m_blocks[m_blocks.size() - 1];
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::extra_bits_number() const noexcept
{
    return bit_index(m_bits_number);
}

template <template <typename> class Vector, typename Block>
constexpr typename dynamic_bitset<Vector, Block>::size_type dynamic_bitset<Vector, Block>::unused_bits_number() const noexcept
{
    return bits_per_block - extra_bits_number();
}

template <template <typename> class Vector, typename Block>
template <typename BinaryOperation>
constexpr void dynamic_bitset<Vector, Block>::apply(const dynamic_bitset<Vector, Block>& other, BinaryOperation binary_op)
{
    assert(num_blocks() == other.num_blocks());
    std::transform(std::cbegin(m_blocks), std::cend(m_blocks), std::cbegin(other.m_blocks), std::begin(m_blocks), binary_op);
}

template <template <typename> class Vector, typename Block>
template <typename UnaryOperation>
constexpr void dynamic_bitset<Vector, Block>::apply(UnaryOperation unary_op)
{
    std::transform(std::cbegin(m_blocks), std::cend(m_blocks), std::begin(m_blocks), unary_op);
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::apply_left_shift(size_type shift)
{
    assert(shift > 0);
    assert(shift < capacity());

    const size_type blocks_shift = shift / bits_per_block;
    const size_type bits_offset  = shift % bits_per_block;

    if (bits_offset == 0) {
        for (size_type i = m_blocks.size() - 1; i >= blocks_shift; --i) { m_blocks[i] = m_blocks[i - blocks_shift]; }
    } else {
        const size_type reverse_bits_offset = bits_per_block - bits_offset;
        for (size_type i = m_blocks.size() - 1; i > blocks_shift; --i) {
            m_blocks[i] = block_type((m_blocks[i - blocks_shift] << bits_offset)
                                     | block_type(m_blocks[i - blocks_shift - 1] >> reverse_bits_offset));
        }
        m_blocks[blocks_shift] = block_type(m_blocks[0] << bits_offset);
    }

    // set bit that came at the right to 0 in unmodified blocks
    std::fill(std::begin(m_blocks),
              std::begin(m_blocks) + static_cast<typename decltype(m_blocks)::difference_type>(blocks_shift),
              zero_block);
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::apply_right_shift(size_type shift)
{
    assert(shift > 0);
    assert(shift < capacity());

    const size_type blocks_shift        = shift / bits_per_block;
    const size_type bits_offset         = shift % bits_per_block;
    const size_type last_block_to_shift = m_blocks.size() - blocks_shift - 1;

    if (bits_offset == 0) {
        for (size_type i = 0; i <= last_block_to_shift; ++i) { m_blocks[i] = m_blocks[i + blocks_shift]; }
    } else {
        const size_type reverse_bits_offset = bits_per_block - bits_offset;
        for (size_type i = 0; i < last_block_to_shift; ++i) {
            m_blocks[i] = block_type((m_blocks[i + blocks_shift] >> bits_offset)
                                     | block_type(m_blocks[i + blocks_shift + 1] << reverse_bits_offset));
        }
        m_blocks[last_block_to_shift] = block_type(m_blocks[m_blocks.size() - 1] >> bits_offset);
    }

    // set bit that came at the left to 0 in unmodified blocks
    std::fill(std::begin(m_blocks) + static_cast<typename decltype(m_blocks)::difference_type>(last_block_to_shift + 1),
              std::end(m_blocks),
              zero_block);
}

template <template <typename> class Vector, typename Block>
constexpr void dynamic_bitset<Vector, Block>::sanitize()
{
    size_type shift = m_bits_number % bits_per_block;
    if (shift > 0) { last_block() &= static_cast<block_type>(~(one_block << shift)); }
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::check_unused_bits() const noexcept
{
    const size_type extra_bits = extra_bits_number();
    if (extra_bits > 0) { return (last_block() & (one_block << extra_bits)) == zero_block; }
    return true;
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::check_size() const noexcept
{
    return blocks_required(size()) == m_blocks.size();
}

template <template <typename> class Vector, typename Block>
constexpr bool dynamic_bitset<Vector, Block>::check_consistency() const noexcept
{
    return check_unused_bits() && check_size();
}

//=================================================================================================
// dynamic_bitset external functions implementations
//=================================================================================================

template <template <typename> class Vector, typename Block>
constexpr bool operator!=(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs)
{
    return !(lhs == rhs);
}

template <template <typename> class Vector, typename Block>
constexpr bool operator<=(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs)
{
    return !(rhs < lhs);
}

template <template <typename> class Vector, typename Block>
constexpr bool operator>(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs)
{
    return rhs < lhs;
}

template <template <typename> class Vector, typename Block>
constexpr bool operator>=(const dynamic_bitset<Vector, Block>& lhs, const dynamic_bitset<Vector, Block>& rhs)
{
    return !(lhs < rhs);
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator&(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs)
{
    dynamic_bitset<Vector, Block> result(lhs);
    return result &= rhs;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator|(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs)
{
    dynamic_bitset<Vector, Block> result(lhs);
    return result |= rhs;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator^(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs)
{
    dynamic_bitset<Vector, Block> result(lhs);
    return result ^= rhs;
}

template <template <typename> class Vector, typename Block>
constexpr dynamic_bitset<Vector, Block> operator-(const dynamic_bitset<Vector, Block>& lhs,
                                                  const dynamic_bitset<Vector, Block>& rhs)
{
    dynamic_bitset<Vector, Block> result(lhs);
    return result -= rhs;
}

template <typename _CharT, typename _Traits, typename Block, template <typename> class Vector>
constexpr std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& os,
                                                          const dynamic_bitset<Vector, Block>& bitset)
{
    // A better implementation is possible
    return os << bitset.template to_string<_CharT, _Traits>();
}

template <typename _CharT, typename _Traits, typename Block, template <typename> class Vector>
constexpr std::basic_istream<_CharT, _Traits>& operator>>(std::basic_istream<_CharT, _Traits>& is,
                                                          dynamic_bitset<Vector, Block>&       bitset)
{
    // A better implementation is possible
    constexpr _CharT                                     zero = _CharT('0');
    constexpr _CharT                                     one  = _CharT('1');
    typename std::basic_istream<_CharT, _Traits>::sentry s(is);
    if (!s) { return is; }

    dynamic_bitset<Vector, Block> reverse_bitset;
    _CharT                        val;
    is.get(val);
    while (is.good()) {
        if (val == one) {
            reverse_bitset.push_back(true);
        } else if (val == zero) {
            reverse_bitset.push_back(false);
        } else {
            is.unget();
            break;
        }
        is.get(val);
    }

    bitset.clear();
    if (!reverse_bitset.empty()) {
        for (typename dynamic_bitset<Vector, Block>::size_type i = reverse_bitset.size() - 1; i > 0; --i) {
            bitset.push_back(reverse_bitset.test(i));
        }
        bitset.push_back(reverse_bitset.test(0));
    }

    return is;
}

template <template <typename> class Vector, typename Block>
constexpr void swap(dynamic_bitset<Vector, Block>& bitset1,
                    dynamic_bitset<Vector, Block>& bitset2) noexcept(noexcept(bitset1.swap(bitset2)))
{
    bitset1.swap(bitset2);
}

} // namespace detail