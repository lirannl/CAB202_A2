#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>
#include <cpu_speed.h>
#include <graphics.h>
#include <macros.h>
#include "lcd_model.h"
#include "usb_serial.h"
//---------------------------------------Macros and definitions---------------------------------

// Inputs
#define buttonR 0
#define buttonL 1
#define joyUp 2
#define joyDown 3
#define joyLeft 4
#define joyRight 5
#define joyPress 6

#define leds(val) ({WRITE_BIT(PORTB, PIN2, val); WRITE_BIT(PORTB, PIN3, val);})

#define COORD_TO_COORD 0
#define OBJ_TO_COORD 1
#define COORD_TO_OBJ 2
#define OBJ_TO_OBJ 3

#define IS_GAME_PAUSED !BIT_VALUE(TIMSK1, TOIE1)
#define GAME_PAUSED(pause_unpause) WRITE_BIT(TIMSK1, TOIE1, !pause_unpause)

#define len(a) (sizeof(a) / sizeof((a)[0]))

#define FLIP_BIT(reg, pin) (reg) ^= (1 << pin); // From my week 7 AMS - question 4

#define STATUS_BAR_HEIGHT 10

//---------------------------------------Types and structs--------------------------------------

// Bitmaps are stored as 25 bits, where 0 represents a BG_COLOUR pixel, and 1 represents an FG_COLOUR pixel. The pixels are meant to be in a 5x5/3x3 pattern
typedef struct
{
    const unsigned long image : 25;
    const uint8_t big : 1;
} tinyBitmap;
tinyBitmap tomBMP = {.big = 0, .image = 0b0000000100010100010000000};
tinyBitmap jerryBMP = {.big = 0, .image = 0b0000001110001000100000000};
tinyBitmap superJerryBMP = {.big = 1, .image = 0b1111100100001000010011000};
tinyBitmap cheeseBMP = {.big = 0, .image = 0b0000000100010100111000000};
tinyBitmap doorBMP = {.big = 1, .image = 0b0111001010010100101001110};
tinyBitmap trapBMP = {.big = 0, .image = 0b0000001110000000111000000};

typedef struct
{ 
    int x;
    int y;
} coords;

typedef struct
{
    coords wallCoords[48]; // Each wall supports up to 30 pixels
    uint8_t len : 7; // Length of wall (in pixels)
    uint8_t valid : 1;
} wall;

struct inputModel
{
    volatile uint8_t buffer :6;
    volatile uint8_t switchClosed :1;
    uint8_t press_handled:1;
};
struct inputModel input[7];

typedef struct
{
    unsigned int lives : 3;
    unsigned int super : 8;
    coords p;
    tinyBitmap *sprite;
} character;
character tom = {.sprite = &tomBMP};
character jerry = {.sprite = &jerryBMP};

typedef struct
{
    unsigned int valid : 1;
    coords p;
    const tinyBitmap *sprite;
} object;

typedef struct
{
    unsigned int valid : 1;
    coords p;
} firework;

struct bitmap_pixels
{
    coords p_array[25];
    unsigned int len;
};

typedef struct
{
    wall walls[6];
    object cheese[5];
    object trap[5];
    firework rocket[20];
    object door;
    uint8_t cheeseCollected;
    uint8_t tom_startx:7;
    uint8_t tom_starty;
    uint8_t jerry_startx:7;
    uint8_t jerry_starty;
    uint8_t finished :1;
} level;

struct game
{
    uint8_t level :7;
    uint8_t score;
    volatile uint8_t wallSpeed;
    volatile uint8_t characterSpeed;
    uint8_t done :1;
};

struct timing // three bytes
{
    volatile unsigned long time : 15;
    volatile unsigned int secondFragments : 7;
    volatile unsigned int secondPassed : 1;
    volatile unsigned int secondFragmentPassed :1;
};

struct timing times = {.time=0, .secondFragments=0};

//---------------------------------------Functions----------------------------------------------

void increment(int *var, int value, int max) // Increment a variable by 1, up to a maximum value, starting from 0 if exceeded
{
    *var = *var + value;
    *var %= max; // Wrap around and start from 0 if greater than the max value
}

void draw_int(int8_t x, int8_t y, int value, colour_t colour) // Helper function - from ams question 2, week 9.
{ 
    char intbuffer[LCD_X];
	snprintf(intbuffer, sizeof(intbuffer), "%d", value);
    draw_string(x, y, "  ", FG_COLOUR); // Padding to prevent digits from previous draws from sticking around
	draw_string(x, y, intbuffer, colour);
}

void draw_formatted(int x, int y, const char * format, ...) { 
    // From the ZDK, with a smaller buffer and a colour.
    va_list args;
    va_start(args, format);
    char buffer[50];
    vsnprintf(buffer, sizeof(buffer), format, args);
    draw_string(x, y, buffer, FG_COLOUR);
}


uint8_t isVacant(uint8_t x, uint8_t y, uint8_t size, level *lvl) // Is this spot available to move to/place in?
{
    if (
    ( y>=STATUS_BAR_HEIGHT&&y<(LCD_Y-size) ) && // Check if within play area vertically
    ( x>=0&&x<(LCD_X-size) ) //&& // Check if within play area horizontally
    )
    return 1; 
    return 0;
}

unsigned int distance(coords *p0, coords *p1, char axis) // Distance calculation program
{
    unsigned int dx = (p0->x > p1->x ? -1 : 1) * (p1->x-p0->x);
    unsigned int dy = (p0->y > p1->y ? -1 : 1) * (p1->y-p0->y);
    if (axis == 'x') return dx;
    else if (axis == 'y') return dy;
    return dx + dy;
}

unsigned int collides_with_wall(level *lvl, coords coord)
{
    for(unsigned int i = 0; i < len(lvl->walls); i++) // For each wall
    {
        if (lvl->walls[i].valid) for(unsigned int pixel = 0; pixel < lvl->walls[i].len; pixel++) // if it's valid - for each pixel of said wall
        {
            //if(distance(&coord, &lvl->walls[i].wallCoords[pixel], COORD_TO_COORD) == 0) return 1; // Stop running as soon as a collision is found (if one is found)
            if (lvl->walls[i].wallCoords[pixel].x == coord.x && lvl->walls[i].wallCoords[pixel].y == coord.y) return 1;
        }        
    }
    return 0;
}

unsigned int bmp_actions(void *optional, int x, int y, const tinyBitmap *bmp, unsigned int mode)
{
    unsigned int collision = 0;
    unsigned int pixels_count = 0;
    uint32_t image = bmp->image;
    int size = (bmp->big) ? 5 : 3; // Big bmps are 5x5, non-big bmps are 3x3
    if (!bmp->big) image = (image >> 6); // Skip the first 6 pixels in non-big bmps, because they're irrelevant in a 3x3 bitmap
    for (int line = -size+1; line < 1; line++) // Numbers are negative because I'm double flipping the sprite to fit in the screen.
    {
        for (int pixel = -size+1; pixel < 1; pixel++)
        {
            if ((image & 1) == 1) // If the current pixel in the sprite is a 1
            {
                if (optional == NULL)
                {
                    if (mode == 0) draw_pixel(x - pixel, y - line, BG_COLOUR); // Clear the sprite's pixel
                    if (mode == 1) draw_pixel(x - pixel, y - line, FG_COLOUR);
                }
                else // If an additional argument is passed
                {
                    coords curr_coords = {.x = x - pixel, .y = y - line};
                    if (mode == 0)
                    {
                        level *lvl = (level*)optional; // lvl is the level optional points to - which it definitely will in my implementation
                        if(collides_with_wall(lvl, curr_coords)) collision = 1; // Collision detected}
                    }
                    else if (mode == 1) // If optional is a pixels list
                    {
                        struct bitmap_pixels *list = (struct bitmap_pixels*)optional;
                        list->p_array[pixels_count] = curr_coords; // Copy the current pixel's coordinates to the array
                        pixels_count++;
                    }
                }
            }
            image = (image >> 1); // Right shift the image to look at the next bit in the next iteration
        }
        if (!bmp->big) image = (image >> 2); // Skip the next 2 bits to reach the next 3x3 line
    }
    if (optional != NULL && mode == 1) // After finishing the list (if that's the operation that was requested), register its length
    {
        struct bitmap_pixels *list = (struct bitmap_pixels*)optional;
        list->len = pixels_count;
    }
    return collision;
}

// Macros to allow calling the different bmp functions as if they're completely separate - they're combined to save on memory.
// Calling bmp drawing/clearing actions to draw with a null pointer since it is not neccesary for drawing
#define draw_bmp(x, y, bmppointer) bmp_actions(NULL, x, y, bmppointer, 1)
#define clear_bmp(x, y, bmppointer) bmp_actions(NULL, x, y, bmppointer, 0)
#define bmp_collides_wall(lvl, x, y, bmppointer) bmp_actions(lvl, x, y, bmppointer, 0)
#define bmp_get_pixels(bmp_pixels, x, y, bmppointer) bmp_actions(bmp_pixels, x, y, bmppointer, 1)

uint8_t isVacantSprite(coords *coordinates, const tinyBitmap *bmp, level *lvl)
{
    if (isVacant(coordinates->x, coordinates->y, bmp->big ? 5 : 3, lvl))
    return !bmp_collides_wall(lvl, coordinates->x, coordinates->y, bmp);
    return 0;
}

uint8_t pixel_lists_have_collision(struct bitmap_pixels *pxbmp0, struct bitmap_pixels *pxbmp1)
{
    for (unsigned int i0 = 0; i0 < pxbmp0->len; i0++) // For each pixel in the first array, loop through all the pixels of the second array
    {
        for (unsigned int i1 = 0; i1 < pxbmp1->len; i1++) // For each pixel of the second array
        {
            if (pxbmp0->p_array[i0].x == pxbmp1->p_array[i1].x && pxbmp0->p_array[i0].y == pxbmp1->p_array[i1].y) return 1; // If there's a collision, stop running
        }
    }
    return 0; // No collision
}

uint8_t bmp_collides_with_coords(coords *p, struct bitmap_pixels *pxbmp)
{
    struct bitmap_pixels single_p = {.len = 1,.p_array[0] = *p};
    return pixel_lists_have_collision(&single_p, pxbmp);
}

uint8_t obj_collides_obj(object *obj0, object *obj1)
{
    if (distance(&obj0->p, &obj1->p, 'a') > 10 || !obj0->valid || !obj1->valid) return 0;
    // Get the lists of pixels
    struct bitmap_pixels obj0_pixels, obj1_pixels;
    bmp_get_pixels(&obj0_pixels, obj0->p.x, obj0->p.y, obj0->sprite);
    bmp_get_pixels(&obj1_pixels, obj1->p.x, obj1->p.y, obj1->sprite);
    return pixel_lists_have_collision(&obj0_pixels, &obj1_pixels);
}

uint8_t character_collides_character(character *c0, character *c1)
{
    // Attempt elimination of possiblity of collision - if the characters are far enough away then there's no possibility of collision
    if (distance(&c0->p, &c1->p, 'a') > 10) return 0;
    // Get the lists of pixels
    struct bitmap_pixels c0_pixels, c1_pixels;
    bmp_get_pixels(&c0_pixels, c0->p.x, c0->p.y, c0->sprite);
    bmp_get_pixels(&c1_pixels, c1->p.x, c1->p.y, c1->sprite);
    return pixel_lists_have_collision(&c0_pixels, &c1_pixels);
}

uint8_t character_collides_obj(character *c, object *obj)
{
    if (!obj->valid) return 0;// Characters can only possibly collide with valid objects
    // Attempt elimination of possiblity of collision - if the characters are far enough away then there's no possibility of collision
    if (distance(&c->p, &obj->p, 'a') > 10) return 0;
    // Get the lists of pixels
    struct bitmap_pixels c_pixels, obj_pixels;
    bmp_get_pixels(&c_pixels, c->p.x, c->p.y, c->sprite);
    bmp_get_pixels(&obj_pixels, obj->p.x, obj->p.y, obj->sprite);
    return pixel_lists_have_collision(&c_pixels, &obj_pixels);
}

uint8_t isTotallyClearObj(object *obj, level *lvl)
{
    unsigned int vacant = 1;
    for (int internali = 0; internali < len(lvl->trap); internali++)
        {
            if (vacant) vacant = !obj_collides_obj(obj, &lvl->trap[internali]);
            if (vacant) vacant = !obj_collides_obj(obj, &lvl->cheese[internali]);
        }
        if (vacant) vacant = !obj_collides_obj(obj, &lvl->door);
    return vacant;
}

void form_wall(wall *w, int x1, int y1, int x2, int y2) { // Build the wall's pixel array
    // This is a modified version of the draw_line function.
    // The formula of x % LCD_X produces an x position which wraps around the screen if out of bounds, for each pixel drawn.
    // The formula of ((y - STATUS_BAR_HEIGHT) % (LCD_Y - STATUS_BAR_HEIGHT)) + STATUS_BAR_HEIGHT does the same idea, only it shifts the original coordinate up by 10 pixels
    // then it wraps around the screen if it was 10 pixels shorter than it was, and then it shifts back down by 10 pixels, because the status bar is at the top, not the bottom.
    // Additionally, it also returns a list of coordinates which the wall occupies
    
    unsigned int pixel_num = 0; // The number of pixels of the wall were formed so far (in this instance of the function)
    
    #define register_wall_pixel(px, py) ({\
        w->wallCoords[pixel_num].x = px % LCD_X; \
        w->wallCoords[pixel_num].y = ((py - STATUS_BAR_HEIGHT) % (LCD_Y - STATUS_BAR_HEIGHT)) + STATUS_BAR_HEIGHT; \
        pixel_num++;})
    
    for (int i = 0; i < len(w->wallCoords); i++) {w->wallCoords[i].x = 0; w->wallCoords[i].y = 0;} // Empty the wall's array with invalid coordinates (y cannot possibly be smaller than 10 since that's the status bar)
	if ( x1 == x2 ) {
		// Draw vertical line
		for ( int i = y1; (y2 > y1) ? i <= y2 : i >= y2; (y2 > y1) ? i++ : i-- ) { // Register wall pixel
            register_wall_pixel(x1 % LCD_X, ((i - STATUS_BAR_HEIGHT) % (LCD_Y - STATUS_BAR_HEIGHT)) + STATUS_BAR_HEIGHT);
		}
	}
	else if ( y1 == y2 ) {
		// Draw horizontal line
		for ( int i = x1; (x2 > x1) ? i <= x2 : i >= x2; (x2 > x1) ? i++ : i-- ) { // Register wall pixel
            register_wall_pixel(i % LCD_X, ((y1 - STATUS_BAR_HEIGHT) % (LCD_Y - STATUS_BAR_HEIGHT)) + STATUS_BAR_HEIGHT);
		}
	}
	else {
		//	Always draw from left to right, regardless of the order the endpoints are 
		//	presented.
		if ( x1 > x2 ) {
			int t = x1;
			x1 = x2;
			x2 = t;
			t = y1;
			y1 = y2;
			y2 = t;
		}

		// Get Bresenhaming...
		float dx = x2 - x1;
		float dy = y2 - y1;
		float err = 0.0;
		float derr = ABS(dy / dx);

		for ( int x = x1, y = y1; (dx > 0) ? x <= x2 : x >= x2; (dx > 0) ? x++ : x-- ) {
            register_wall_pixel(x % LCD_X, ((y - STATUS_BAR_HEIGHT) % (LCD_Y - STATUS_BAR_HEIGHT)) + STATUS_BAR_HEIGHT);
			err += derr;
			while ( err >= 0.5 && ((dy > 0) ? y <= y2 : y >= y2) ) {
                register_wall_pixel(x % LCD_X, ((y - STATUS_BAR_HEIGHT) % (LCD_Y - STATUS_BAR_HEIGHT)) + STATUS_BAR_HEIGHT);
				y += (dy > 0) - (dy < 0);
				err -= 1.0;
			}
		}
	}
    #undef register_wall_pixel // Ensure that this macro only expands within this function to prevent compiler errors later
    w->len = pixel_num; // Register the wall's length based on how many pixels would've normally been drawn by a draw_line function
}

/*coords getNextPosition(coords p, double slope, int8_t vert) // For a given slope/vertical line, get the next position.
{
    coords output = p;
    if (vert != 0) {output.x = p.x; output.y = p.y+vert;} // Manually handling vertical lines since there's no mathematical expression for their
    else // slope is relevant 
    {

    }
    return output;
}*/

void ddrSetup()
{
    SET_INPUT(DDRB, PIN0); // Centre
    SET_INPUT(DDRB, PIN1); // Left
    SET_INPUT(DDRB, PIN7); // Down
    SET_INPUT(DDRD, PIN0); // Right
    SET_INPUT(DDRD, PIN1); // Up
    CLEAR_BIT(DDRF, PIN6); // Left Button
    CLEAR_BIT(DDRF, PIN5); // Right Button
    // Both LEDs
    SET_OUTPUT(DDRB, PIN2);
    SET_OUTPUT(DDRB, PIN3);
}

void setup(void) {
	set_clock_speed(CPU_8MHz);
	lcd_init(LCD_DEFAULT_CONTRAST);
	lcd_clear();
	//	Initialise Timer 0 in normal mode so that it overflows 
	//	with a period of approximately 0.008 seconds.
    //----------Timer 0------------
        CLEAR_BIT(TCCR0B, WGM02);
        // Prescaler
        WRITE_BIT(TCCR0B,CS02, 1);
        WRITE_BIT(TCCR0B,CS01, 0);
        WRITE_BIT(TCCR0B,CS00, 0);
        //	Enable timer 0 interrupt.
        SET_BIT(TIMSK0, TOIE0);
    //----------Timer 1----------- - overflows every 0.008192 seconds (roughly 122 times per second)
        CLEAR_BIT(TCCR1B, WGM12);
        CLEAR_BIT(TCCR1B, WGM13);
        // Prescaler 001
        WRITE_BIT(TCCR1B, CS12, 0);
        WRITE_BIT(TCCR1B, CS11, 0);
        WRITE_BIT(TCCR1B, CS10, 1);
        // Enable timer 1 interrupt
        SET_BIT(TIMSK1, TOIE1);
	//	Enable interrupts.
    sei();
    // Enable LCD backlight
    SET_BIT(PORTC, PIN7);
    ddrSetup();
}

void debounce_switch(unsigned int index) // Modified from week 9 AMS, question 3
{
    input[index].buffer = (input[index].buffer << 1); 
    //		(h.b) Bitwise AND with a mask in which the 5 bits on the right
    //			are 1 and the others are 0.
    input[index].buffer &= 0b00011111;
    //		(h.c) Use bitwise OR to add the current open/closed value of the 
    //			left button switch to the history.
    switch (index)
    {
        case buttonR:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PINF, PIN5));
            break;
        case buttonL:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PINF, PIN6));
            break;
        case joyUp:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PIND, PIN1));
            break;
        case joyDown:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PINB, PIN7));
            break;
        case joyLeft:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PINB, PIN1));
            break;
        case joyRight:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PIND, PIN0));
            break;
        case joyPress:
            WRITE_BIT(input[index].buffer, 0, BIT_VALUE(PINB, PIN0));
        default:
            break;
    }
    //		    (If input[index].buffer is equal to the bit mask, then the switch has been 
    //			observed 5 times in a row to be closed. Assign the value 1 to 
    //			prevState, indicating that the switch should now be considered to be
    //			officially "closed".
    if (input[index].buffer == 0b00011111) input[index].switchClosed = 1;
    //		(h.e) If input[index].buffer is equal to 0, then the switch has been observed 
    //			to be open at least 5 in a row, so store 0 in prevState, 
    //			indicating that the switch should now be considered to be officially "closed".
    if (input[index].buffer == 0) input[index].switchClosed = 0;
}

void debounce_process(void) {
    for (int i = 0; i < len(input); i++) // For each input
    if (!input[i].switchClosed) input[i].press_handled = 0; // Reset the single press handling whenever the switch is released
}

ISR(TIMER0_OVF_vect) { // Timer 0 overflows - debounce switches
    for (int i = 0; i < 7; i++) debounce_switch(i);
}

ISR(TIMER1_OVF_vect) { // Timer 1 overflows - increment game time
    times.secondFragments++;
    times.secondFragmentPassed = 1; // Register that a second fragment passed
    if (times.secondFragments == 122) 
    {
    times.time++; 
    _delay_ms(0.576); // Make the increment of time happen as close to 1hz as possible
    times.secondFragments = 0;
    times.secondPassed = 1; // Register that a second just passed
    }
}

uint8_t moveCharacter(int8_t xoffset, int8_t yoffset, character *c, level *lvl)
{
    coords targetCoords = {.x = c->p.x + xoffset, .y = c->p.y + yoffset};
    if
    (
        isVacantSprite(&targetCoords, c->sprite, lvl)
        &&
        isVacant(c->p.x + xoffset, c->p.y + yoffset, c->sprite->big ? 5 : 3, lvl)
    )
    {
        clear_bmp(c->p.x, c->p.y, c->sprite); // Clear the character sprite from the previous position
        c->p.x+=xoffset; c->p.y+=yoffset;
        return 1; // Successful movement
    }
    return 0; // Movement failed
}

uint8_t tryMoveFirework(int xoffset, int yoffset, firework *obj, level *lvl)
{
    if (isVacant(obj->p.x, obj->p.y, 1, lvl) && !collides_with_wall(lvl, obj->p))
    {
        draw_pixel(obj->p.x, obj->p.y, BG_COLOUR); // Remove the firework from the current position
        // Change the firework's position
        obj->p.x += xoffset;
        obj->p.y += yoffset;
        return 1;
    }
    return 0;
}

uint8_t moveTowards(char axis, firework *obj, coords *target, level *lvl) // Taken from my assignment 1 (and adapted)
{
    
    uint8_t successful = 0;
    if (axis == 'x')
    {
        if (obj->p.x < target->x) successful = tryMoveFirework(1, 0, obj, lvl);
        else if (obj->p.x > target->x) successful = tryMoveFirework(-1, 0, obj, lvl);
    }
    else if (axis == 'y')
    {
        if (obj->p.y < target->y) successful = tryMoveFirework(0, 1, obj, lvl);
        else if (obj->p.y > target->y) successful = tryMoveFirework(0, -1, obj, lvl);
    }
    return successful;
}

void moveFireWorks(level *lvl) // Taken from my assignment 1 (and adapted)
{
    coords target = {.x = tom.p.x + 1, .y = tom.p.y + 1}; // Aiming the rockets at tom's centre (as opposed to his origin - top left corner)
    for (int i = 0; i < len(lvl->rocket); i++)
    {
        if (lvl->rocket[i].valid) // Only bother trying to move valid rockets
        {
            if ((rand() % 2 == 0 || distance(&target, &lvl->rocket[i].p, 'y') == 0) && distance(&target, &lvl->rocket[i].p, 'x') > 0) // 50% chance of following tom over X (or if the rocket is on Tom's y coord)
            {
                lvl->rocket[i].valid = moveTowards('x', &lvl->rocket[i], &target, lvl);
            }
            else // 50% chance of following tom over y (or if the rocket is on Tom's x coord)
            {
                lvl->rocket[i].valid = moveTowards('y', &lvl->rocket[i], &target, lvl);
            }
        }
    else draw_pixel(lvl->rocket[i].p.x, lvl->rocket[i].p.y, BG_COLOUR); // Remove invalid rockets from the screen
    }
}

coords getStartingCoords(character *character, level *lvl)
{
    coords coordinates = {.x = 0, .y = 0}; // Initialise
    if (character == &jerry)
    {
        coordinates.x = lvl->jerry_startx; coordinates.y = lvl->jerry_starty;
    }
    else if (character == &tom)
    {
        coordinates.x = lvl->tom_startx; coordinates.y = lvl->tom_starty;
    }
    return coordinates;
}

void respawnCharacter(character *character, level *lvl) // Character respawn - guaranteed to succeed because it'll look in 4 directions until it finds a free spot
{
    unsigned int placed = 0;
    #define move(target, xoffset, yoffset) ({coords newTarget = {.x = target.x + xoffset, .y = target.y + yoffset};\
    if (isVacantSprite(&newTarget, character->sprite, lvl)) { \
    clear_bmp(character->p.x, character->p.y, character->sprite); \
    character->p = newTarget; placed = 1;}})
    int xoffset;
    int yoffset;
    coords target = getStartingCoords(character, lvl);
    if (!bmp_collides_wall(lvl, target.x, target.y, character->sprite)) move(target, 0, 0);
    for (yoffset = 0; !placed && target.y + yoffset > STATUS_BAR_HEIGHT ; yoffset--) // Try spawning up if occupied
    { move(target, 0, yoffset); }
    for (yoffset = 0; !placed && target.y + yoffset < LCD_Y ; yoffset++) // Try spawning down if still occupied
    { move(target, 0, yoffset); }
    for (xoffset = 0; !placed && target.x + xoffset < LCD_X ; xoffset++) // Try spawning left if still occupied
    { move(target, xoffset, 0); }
    for (xoffset = 0; !placed && target.x + xoffset > 0 ; xoffset--) // Try spawning right if still occupied
    { move(target, xoffset, 0); }
    #undef move
}

void fireFirework(level *lvl, struct game *data) // Taken from my first assignment
{
    if (data->score >= 3) for (int i = 0; i < len(lvl->rocket); i++) // Before fireworks are enabled, the only way to gain score is by eating cheese
    {
        if (!lvl->rocket[i].valid) 
            {
                lvl->rocket[i].p = jerry.p;
                lvl->rocket[i].valid = 1;
                break;
            }
    }
}

void readControls(level *lvl, struct game *data)
{
    if (input[joyLeft].switchClosed) moveCharacter(-1, 0, &jerry, lvl);
    if (input[joyRight].switchClosed) moveCharacter(1, 0, &jerry, lvl);
    if (input[joyUp].switchClosed) moveCharacter(0, -1, &jerry, lvl);
    if (input[joyDown].switchClosed) moveCharacter(0, 1, &jerry, lvl);
    if (input[joyPress].switchClosed && !input[joyPress].press_handled) // Once per joystick press
    { fireFirework(lvl, data); input[joyPress].press_handled = 1; }
    if (input[buttonL].switchClosed && !input[buttonL].press_handled) 
    {lvl->finished = 1; input[buttonL].press_handled = 1;}
    // For every click of the right button - toggle a pause (ignoring the first second of each game - since the player just clicked it to proceed)
    if (input[buttonR].switchClosed && !input[buttonR].press_handled && times.secondFragments > 2) 
    {FLIP_BIT(TIMSK1, TOIE1); input[buttonR].press_handled = 1;}
}

void checkCollisions(level *lvl, struct game *data)
{
    if (character_collides_character(&tom, &jerry)) // Upon collision with tom;
    {
        respawnCharacter(&jerry, lvl);
        respawnCharacter(&tom, lvl);
        jerry.lives--;
    }
    for (int i = 0; i < len(lvl->cheese); i++) // For each cheese and trap
    {
        if (character_collides_obj(&jerry, &lvl->cheese[i])) // Handling cheese collision
        {
            data->score++;
            lvl->cheeseCollected++;
            lvl->cheese[i].valid = 0;
            clear_bmp(lvl->cheese[i].p.x, lvl->cheese[i].p.y, lvl->cheese[i].sprite);
        }
        if (character_collides_obj(&jerry, &lvl->trap[i])) // Handling trap collision
        {
            jerry.lives--;
            lvl->trap[i].valid = 0;
            clear_bmp(lvl->trap[i].p.x, lvl->trap[i].p.y, lvl->trap[i].sprite);
        }
    }
    struct bitmap_pixels tom_pixels;
    bmp_get_pixels(&tom_pixels, tom.p.x, tom.p.y, tom.sprite);
    for (int i = 0; i < len(lvl->rocket); i++) // For each firework rocket
    {
        if (lvl->rocket[i].valid && bmp_collides_with_coords(&lvl->rocket[i].p, &tom_pixels)) // If tom collides with a firework
        {
            respawnCharacter(&tom, lvl);
            lvl->rocket[i].valid = 0;
            data->score++;
        }
    }
}

void startingScreen()
{
    clear_screen();
    draw_string(0, 0, "Liran", FG_COLOUR);
    draw_string(0, 10, "n10446711", FG_COLOUR);
    draw_string(0, 20, "TJ", FG_COLOUR);
    show_screen();
    _delay_ms(500);
    while(!(input[buttonR].switchClosed && !input[buttonR].press_handled)) {debounce_process();} // Wait until buttonR is pressed
    input[buttonR].press_handled = 1;
}

void gameOverScreen()
{
    clear_screen();
    draw_string(0, 0, "Game over!", FG_COLOUR);
    draw_string(0, 10, "Press SW3", FG_COLOUR);
    draw_string(0, 20, "To restart.", FG_COLOUR);
    show_screen();
    while(!(input[buttonR].switchClosed && !input[buttonR].press_handled)) debounce_process(); // Wait until buttonR is pressed
    input[buttonR].press_handled = 1;
}

void levelInitUSB(level *lvl, struct game *data)
{
    clear_screen();
    draw_string(0, 0, "Please enter next", FG_COLOUR);
    draw_string(0, 10, "level via serial", FG_COLOUR);
    show_screen();
    while(!(input[buttonR].switchClosed && !input[buttonR].press_handled)) debounce_process(); // Wait until buttonR is pressed
    input[buttonR].press_handled = 1;
    data->done = 1; // Game completed - finish game and start a new one
}

object makeDef(tinyBitmap *bmp) // Object initialiser
    { 
        object returnval = {.valid = 0, .sprite = bmp}; return returnval;
    }

void levelInit(struct game *data, level *thisLevel)
{
    thisLevel->finished = 0;
    if (data->level == 1) // When initiating level 1 -
    {
        thisLevel->jerry_startx = 0; thisLevel->jerry_starty = STATUS_BAR_HEIGHT + 1;
        thisLevel->tom_startx = LCD_X - 5; thisLevel->tom_starty = LCD_Y - 9;
        thisLevel->walls[0].valid = 1;
        thisLevel->walls[1].valid = 1;
        thisLevel->walls[2].valid = 1;
        thisLevel->walls[3].valid = 1;
        thisLevel->walls[4].valid = 0;
        thisLevel->walls[5].valid = 0;
        // Hardcoded walls
        {
            form_wall(&thisLevel->walls[0], 18, 15, 13, 25);
            form_wall(&thisLevel->walls[1], 25, 35, 25, 45);
            form_wall(&thisLevel->walls[2], 45, 10, 60, 10);
            form_wall(&thisLevel->walls[3], 58, 25, 72, 30);
        }
    }
    if (data->level == 2)
    {
        usb_init(); // Enable serial
    }
    if (data->level > 1) // After level 1
    {
        for (int i = 0; i < 6; i++) thisLevel->walls[i].valid = 0;
        levelInitUSB(thisLevel, data); // Initialise level from USB
    }
    // Regardless of which level it is - move tom and Jerry to their starting positions
    respawnCharacter(&jerry, thisLevel);
    respawnCharacter(&tom, thisLevel);
    // Initilaise objects
    for(int i = 0; i < 5; i++)
    {
        thisLevel->cheese[i] = makeDef(&cheeseBMP);
        thisLevel->trap[i] = makeDef(&trapBMP);
    }
    for(int i = 0; i < 20; i++) thisLevel->rocket[i].valid = 0;
    thisLevel->door = makeDef(&doorBMP);
    clear_screen();
}

void placeObj(level *lvl, object *obj)
{
    // Generate a random target location (within play area)
    unsigned int size = obj->sprite->big ? 5 : 3;
    coords target;
    target.x = rand() % LCD_X; target.y = ( rand() % (LCD_Y - STATUS_BAR_HEIGHT) ) + STATUS_BAR_HEIGHT;
    // Keep on generating random locations until a vacant one is found
    while(!isVacant(target.x, target.y, size, lvl) || 
    !isVacantSprite(&target, obj->sprite, lvl) || 
    !isTotallyClearObj(obj, lvl) ||
    distance(&target, &tom.p, 'a') < 10) // Don't place the cheese too close to tom
    {
        target.x = rand() % LCD_X; target.y = ( rand() % (LCD_Y - STATUS_BAR_HEIGHT) ) + STATUS_BAR_HEIGHT;
    }
    obj->p = target;
    obj->valid = 1;
}

void placeCheese(level *lvl)
{
    for (int i = 0; i < len(lvl->cheese); i++) // For each potential cheese
    if (!lvl->cheese[i].valid) // Try to find a vacant cheese slot (effectively limiting the max number of cheeses in a level to 5)
    {
        placeObj(lvl, &lvl->cheese[i]);
        return; // Stop searching as soon as a vacant slot is found
    }
    // If a vacant cheese slot isn't found, nothing is done.
}

void placeTrapAttempt(level *lvl)
{
    for (int i = 0; i < len(lvl->trap); i++) // For each potential cheese
    if (!lvl->trap[i].valid) // Try to find a vacant trap slot (effectively limiting the max number of traps in a level to 5)
    {
        object target = {.p = tom.p, .valid = 1, .sprite = &trapBMP}; // Create a virtual object at the target location
        if (isTotallyClearObj(&target, lvl)) 
        {
            lvl->trap[i].p = tom.p; 
            lvl->trap[i].valid = 1;
        } // Place a trap at tom's position if free
        return; // Stop searching as soon as a vacant slot is found
    }
}

void timed_events(level *lvl)
{
    if (times.secondPassed)
    {
        if (times.time % 2 == 0) placeCheese(lvl);
        if (times.time % 3 == 0) placeTrapAttempt(lvl);
        times.secondPassed = 0; // Reset secondPassed, since the events for this second have now already been completed
    }
    if (times.secondFragmentPassed)
    {
        times.secondFragmentPassed = 0;
    }
}

void draw_statusbar(level *lvl, struct game *data)
{
    draw_formatted(0, 0, "LV%d|L%d|S%02d|%02d:", data->level, jerry.lives, data->score, times.time / 60);
    draw_formatted(69, 0, "%02d", times.time%60);
}

void draw_walls(level *level, unsigned int show)
{
    for (int i = 0; i < len(level->walls); i++) // For each wall
    if (level->walls[i].valid) for(int wallPixel = 0; wallPixel < level->walls[i].len; wallPixel++) // If it is valid, then for each of its' pixels
    if (level->walls[i].wallCoords[wallPixel].y >= 10) // If within play area
    {
        draw_pixel(level->walls[i].wallCoords[wallPixel].x, level->walls[i].wallCoords[wallPixel].y, show ? FG_COLOUR : BG_COLOUR); // Either draw or clear it (depending on the whether show is set or not)
    }
}

void draw_objects(level *level)
{
    draw_bmp(jerry.p.x, jerry.p.y, jerry.sprite);
    draw_bmp(tom.p.x, tom.p.y, tom.sprite);
    for (int i = 0; i < len(level->cheese); i++)
    {
        if (level->cheese[i].valid) draw_bmp(level->cheese[i].p.x, level->cheese[i].p.y, level->cheese[i].sprite);
        if (level->trap[i].valid) draw_bmp(level->trap[i].p.x, level->trap[i].p.y, level->trap[i].sprite);
    }
    if (level->door.valid) draw_bmp(level->door.p.x, level->door.p.y, level->door.sprite);
    for (int i = 0; i < len(level->rocket); i++)
    if (level->rocket[i].valid) // Only attempt to draw valid rockets
    {
        coords rocketCoords = level->rocket[i].p;
        draw_pixel(rocketCoords.x, rocketCoords.y, FG_COLOUR);
    }
}


void process(struct game *data, level *level) // Game tick
{
    debounce_process();
    if (jerry.lives == 0) {data->done = 1; return;} // If there are 0 lives left, game over.
    if (!IS_GAME_PAUSED && times.time > 0) timed_events(level); // Timed events don't occur on the 0th second, nor do they occur while the game is paused
    static int scaler = 0; // Establish a scaler for use with the moveFireWorks function
    increment(&scaler, 1, 3); if (scaler == 0 && !IS_GAME_PAUSED)
    moveFireWorks(level);
    readControls(level, data);
    checkCollisions(level, data);
    draw_statusbar(level, data);
    draw_walls(level, 1); // Redraw the walls
    draw_objects(level);
    if (!data->done) show_screen();
    _delay_ms(5);
}



void game() // Run a game
{
    struct game gameData;
    gameData.done = 0;
    gameData.score = 0;
    gameData.characterSpeed = 1;
    gameData.wallSpeed = 0;
    tom.lives = 5;
    tom.super = 0;
    jerry.lives = 5;
    jerry.super = 0;
    // Reset the time and unpause (in case the previous game was paused), also, reset the second indicator
    times.time = 0;
    times.secondFragments = 0;
    times.secondPassed = 0;
    times.secondFragmentPassed = 0;
    GAME_PAUSED(0);
    for(gameData.level = 1; !gameData.done; gameData.level++) // After a game is set up, run the current game until a game over - level after level
    {
        level CurrLevel;
        levelInit(&gameData, &CurrLevel);
        while (!gameData.done && !CurrLevel.finished)
        process(&gameData, &CurrLevel);
    }
    gameOverScreen();
}

int main(void) {
	setup();
    startingScreen();
    srand(times.secondFragments); // Seed rand() with the current second fragments value - 
	for ( ;; ) {
		game();
	}
}
