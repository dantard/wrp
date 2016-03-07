#define CLEAR_BIT(array, bit) (array &= ~(1 << bit))
#define CHECK_BIT(array, bit) ((array >> bit) & 1)
#define SET_BIT(array, bit)   (array |= (1 << bit))
#define SET_BIT_DRY(array, bit)   (array | (1 << bit))
