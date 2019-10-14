void tomRandom(struct game *data)
{
    // Randomly select a direction in the next 2 lines
    tom.bresen.direction = (rand() % 2 == 0) ? 1 : -1;
    tom.bresen.derr = ((rand() % 2 == 0) ? 1 : -1) * (rand() % 1000 + 1) / 100;
    data->characterSpeed = ((rand() % 16) + 8) / 4; // Select a random, appropriate speed 
}

coords getNextPosition(coords p, bresenhaming *bresen) // For a given slope/vertical line, get the next position.
{
    float slope = bresen->derr;
    if (slope != 0) slope = 1/slope; // get a perpendicular slope to the one provided
    coords output = p;
    if (bresen->vert) {output.x = p.x; output.y = p.y+bresen->direction;} // Manually handling vertical lines since there's no mathematical expression for their slope
    else // slope is relevant 
    {
        // The bresenhaming algorithm from draw_wall, adapted to work for a single iteration with a slope
        if ( bresen->err >= 0.5 ) { 
				output.y += bresen->direction;
				bresen->err -= 1.0;
			}
        else {
			output.x += bresen->direction;
			bresen->err += slope;
		}
    }
    return output;
}

// Calculate wall slopes (on levelinit)
    for(int i = 0; i < len(thisLevel->walls); i++)
    {
        wall *currWall = &thisLevel->walls[i];
        currWall->bresen.err = 0;
        currWall->bresen.direction = (rand() % 2 == 0) ? 1 : -1; // Select a direction randomly, since I'm not implementing ADC
        if ( (currWall->wallCoords[currWall->len].x) != (currWall->wallCoords[0].x) ) // If the first and last x values aren't equal (non vertical wall)
        // Calculate the wall's slope
        currWall->bresen.derr = ( (currWall->wallCoords[currWall->len].y) - (currWall->wallCoords[0].y) )/( (currWall->wallCoords[currWall->len].x) - (currWall->wallCoords[0].x) );
        else // Vertical wall
        {
            currWall->bresen.vert = 1; currWall->bresen.derr = 0;
        }
        
    }