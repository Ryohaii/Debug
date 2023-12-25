#include "TilesWorld.hpp"

TilesWorld::TilesWorld() : mView(mView){}

TilesWorld::TilesWorld(int biomeSizeX, int biomeSizeY, std::shared_ptr<sf::View>& view) : mView(view)
{
    if (!waterShader.loadFromFile("vague.vert", "vague.frag"))
    {
        std::cout << "erreur shader vague!" << std::endl;
    }

    if (!mFont.loadFromFile("data/JMH_Typewriter.ttf")) {
        // Gérer l'erreur
    }

    // Redimensionnement du tableau extérieur
    biomes.resize(biomeSizeX);

    // Redimensionnement de chaque sous-vecteur
    for (int i = 0; i < biomeSizeX; ++i) {
        biomes[i].resize(biomeSizeY);
        for (int j = 0; j < biomeSizeY; ++j) {
            // Initialisation de chaque élément à nullptr
            biomes[i][j] = nullptr;
        }
    }

    // Vérifications pour s'assurer que le tableau a été correctement initialisé
    assert(biomes.size() == biomeSizeX);
    for (int i = 0; i < biomeSizeX; ++i) {
        assert(biomes[i].size() == biomeSizeY);

        std::cout << "Init biomes biomes.size(): " << biomes.size() << " biomes[i].size(): " << biomes[i].size() << std::endl;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::cout << "Init world" << std::endl;
}

int TilesWorld::getWidth()
{
    if (mapWidth == 0) {
        std::cout << "Warning: width is zero. Did you forget to initialize it?" << std::endl;
    }
    return mapWidth;
}

int TilesWorld::getHeight()
{
    if (mapHeight == 0) {
        std::cout << "Warning: height is zero. Did you forget to initialize it?" << std::endl;
    }
    return mapHeight;
}

void TilesWorld::initMapConfig(int mapWidth, int mapHeight, int biomeSizeX, int biomeSizeY, int tileSize)
{
    // Initialisation
    this->mapWidth = mapWidth;
    this->mapHeight = mapHeight;
    this->biomeSizeX = biomeSizeX;
    this->biomeSizeY = biomeSizeY;
    this->tileSize = tileSize;
}

TilesWorld::~TilesWorld()
{
    for (auto& row : biomes) {
        row.clear();
    }
    biomes.clear();
}

std::vector<Tile>& TilesWorld::getTiles(int layer)
{
    return mLayers[layer];
}

bool TilesWorld::hasLayer(int layer) {
    return mLayers.find(layer) != mLayers.end();
}

bool TilesWorld::hasTilesInLayer(int layer) {
    auto it = mLayers.find(layer);
    if (it != mLayers.end()) {
        return !it->second.empty();
    }
    return false;
}

std::map<int, std::vector<Tile>>& TilesWorld::getAllLayers()
{
    return mLayers;
}

bool TilesWorld::checkCollision(const sf::Image& mask1, const sf::Vector2f& pos1, const sf::Image& mask2, const sf::Vector2f& pos2)
{
    sf::Vector2u size1 = mask1.getSize();
    sf::Vector2u size2 = mask2.getSize();

    // Calculez la zone de chevauchement
    int xStart = std::max(pos1.x, pos2.x);
    int yStart = std::max(pos1.y, pos2.y);
    int xEnd = std::min(pos1.x + size1.x, pos2.x + size2.x);
    int yEnd = std::min(pos1.y + size1.y, pos2.y + size2.y);

    //std::cout << "pos1: (" << pos1.x << ", " << pos1.y << "), size1: (" << size1.x << ", " << size1.y << ")" << std::endl;
    //std::cout << "pos2: (" << pos2.x << ", " << pos2.y << "), size2: (" << size2.x << ", " << size2.y << ")" << std::endl;

    /*std::cout << "xStart: " << xStart << ", yStart: " << yStart << ", xEnd: " << xEnd << ", yEnd: " << yEnd << std::endl;*/



    for (int x = xStart; x < xEnd; ++x) {
        for (int y = yStart; y < yEnd; ++y) {
            // Obtenez les pixels correspondants dans les masques
            sf::Color color1 = mask1.getPixel(x - pos1.x, y - pos1.y);
            sf::Color color2 = mask2.getPixel(x - pos2.x, y - pos2.y);
            // Si les deux pixels ne sont pas transparents, il y a une collision
            if (color1.a != 0 && color2.a != 0) {
                //std::cout << "checkCollision :: true" << std::endl;
                return true;
            }
        }
    }
    return false;
}

bool TilesWorld::isCollidable(int x, int y, int width, int height, int layer)
{
    // Vérifiez si l'indice de la couche est valide
    if (layer >= mLayers.size() || layer < 0)
    {
        std::cout << "Indice de la couche invalide: " << layer << std::endl;
        return false; // ou lancez une exception
    }

    // Trouver la tuile à la position (x, y) pour la couche spécifiée
    Tile* tile = getTileAt(x, y, width, height, layer); // Supposons que getTileAt retourne un pointeur, null si non trouvé

    // Vérifiez si la tuile est valide
    if (tile == nullptr)
    {
        //std::cout << "Tuile non trouvée à la position (" << x << ", " << y << ") pour la couche " << layer << std::endl;
        return false; // ou lancez une exception
    }

    return tile->getCollidable();
}

Tile* TilesWorld::getTileAt(int x, int y, int width, int height, int layer)
{
    // Vérifiez si l'indice de la couche est valide
    if (layer >= mLayers.size() || layer < 0)
    {
        //std::cout << "Indice de la couche invalide: " << layer << std::endl;
        return nullptr; // ou lancez une exception
    }

    // Parcourir toutes les tuiles de la couche spécifiée
    for (Tile& tile : mLayers[layer])
    {
        int tileX = static_cast<int>(tile.getPosition().x); // Remplacez par la méthode appropriée pour obtenir la position X de la tuile
        int tileY = static_cast<int>(tile.getPosition().y); // Remplacez par la méthode appropriée pour obtenir la position Y de la tuile

        // Vérifiez si la position (x, y) est à l'intérieur de cette tuile
        if (x >= tileX && x <= tileX + width && y >= tileY && y <= tileY + height)
        {
            return &tile;
        }
    }

    // Si aucune tuile correspondante n'est trouvée, retournez nullptr
    return nullptr;
}

void TilesWorld::loadTexturesFromJson(const nlohmann::json& j, const std::string& key)
{
    for (const auto& tileInfo : j[key])
    {
        int id = tileInfo["id"];
        std::string spriteFile = tileInfo["spriteFile"];
        bool collidable = tileInfo["collidable"];  // Ajout de cette ligne

        sf::Texture texture;
        if (!texture.loadFromFile(spriteFile)) {
            std::cout << "Erreur lors du chargement de la texture" << std::endl;
            return;
        }
        texture.setSmooth(true);
        textures[id] = texture;

        if (collidable)  // Ajout de cette condition
        {
            std::string collisionMaskFile = tileInfo["collisionMask"];

            sf::Image collisionMaskTexture;
            if (!collisionMaskTexture.loadFromFile(collisionMaskFile)) {
                std::cout << "Erreur lors du chargement du masque de collision" << std::endl;
                return;
            }
            collisionMaskTextures[id] = collisionMaskTexture;
        }
    }
}

void TilesWorld::loadTilesConfigs() {
    //std::cout << "TilesWorld::loadTilesConfigs()" << std::endl;

    std::ifstream configFile("data/config/tiles_save.json");
    nlohmann::json tileJsonData;
    configFile >> tileJsonData;

    // Charger les textures et masques de collision
    if (tileJsonData.find("All_Tiles") != tileJsonData.end()) {
        loadTexturesFromJson(tileJsonData, "All_Tiles");  // Remplacez "All_Tiles" par la clé JSON réelle
    }

    for (const auto& tileInfo : tileJsonData["All_Tiles"]) {
        TilesConfig config;
        config.id = tileInfo["id"];
        config.name = tileInfo["name"];
        //config.spriteFile = tileInfo["spriteFile"];
        //config.collisionMask = tileInfo["collisionMask"];
        config.canSpawnBiome = tileInfo["canSpawnBiome"].get<std::vector<int>>();
        config.typeTile = tileInfo["typeTile"];
        config.rarity = tileInfo["rarity"];
        config.collidable = tileInfo["collidable"];
        config.mineral = tileInfo["mineral"];
        config.sizeX = tileInfo["sizeX"];
        config.sizeY = tileInfo["sizeY"];
        config.posX = tileInfo["posX"];
        config.posY = tileInfo["posY"];

        tilesDB.push_back(config);
    }
}

void TilesWorld::update(float deltaTime)
{
    // Parcourir chaque couche
    for (auto& layerPair : mLayers) {
        int layerIndex = layerPair.first;
        std::vector<Tile>& tiles = layerPair.second;

        // Parcourir chaque tuile de la couche
        for (Tile& tile : tiles) {
            if (tile.isActive)
            {
                tile.update(deltaTime);
            }
        }
    }
}

void TilesWorld::draw(sf::RenderWindow& window, sf::View& view, sf::Time elapsed)
{
    float time = elapsed.asSeconds(); // Convertir sf::Time en secondes
    // Parcourir chaque couche
    for (auto& layerPair : mLayers) {
        int layerIndex = layerPair.first;
        std::vector<Tile>& tiles = layerPair.second;

        // Parcourir chaque tuile de la couche
        for (Tile& tile : tiles) {
            if(tile.name == "Tree 1" || tile.name == "Tree 2" || tile.name == "Tree 3")
            {
                // Obtenez la position de la souris en coordonnées de la vue
                sf::Vector2i mousePosScreen = sf::Mouse::getPosition(window);
                sf::Vector2f mousePosView = window.mapPixelToCoords(mousePosScreen, view);

                // Créez une sf::FloatRect pour représenter la position de la souris
                sf::FloatRect mouseFinalPos(mousePosView.x, mousePosView.y, 1, 1);

                if (tile.checkCollision(mouseFinalPos))
                {
                    tile.highlight();
                }
                else
                {
                    tile.resetHighlight();
                }
            }

            if (tile.isActive && layerIndex == 0)
            {
                
                waterShader.setUniform("time", time); // Passer le temps au shader
                tile.draw(window, waterShader, elapsed);
            }
        }
    }
}

void TilesWorld::drawObjets(sf::RenderWindow& window, sf::View& view, sf::Time elapsed)
{
    // Parcourir chaque couche
    for (auto& layerPair : mLayers)
    {
        int layerIndex = layerPair.first;
        std::vector<Tile>& tiles = layerPair.second;

        if(layerIndex == 1)
        {
            // Parcourir chaque tuile de la couche
            for (Tile& tile : tiles)
            {
                if (tile.isActive && layerIndex == 1)
                {
                    //window.draw(/* votre tuile */, &shader);
                    tile.draw(window, waterShader, elapsed);
                }
            }
        }
    }

}

void TilesWorld::setEntityMapPosition(const std::shared_ptr<Entity>& mEntity, const sf::Vector2f& position)
{
    m_entityMapPositions[mEntity] = position;
}

sf::Vector2f TilesWorld::getEntityMapPosition(const std::shared_ptr<Entity>& mEntity) const
{
    auto it = m_entityMapPositions.find(mEntity);
    if (it != m_entityMapPositions.end()) {
        return it->second;
    }
    return sf::Vector2f(0, 0);  // Retournez une position par défaut si l'entité n'est pas trouvée
}

sf::Vector2f TilesWorld::getResourceOresMapPosition(const Resources* resource) const
{
    //auto it = m_resourcesMapPositions.find(resource); // Pas besoin de const_cast ici
    //if (it != m_resourcesMapPositions.end()) {
    //    return it->second;
    //}
    //return sf::Vector2f(0, 0); // Retourne une position par défaut si la ressource n'est pas trouvée
}

bool TilesWorld::isMineralVein(const sf::Vector2f& position) const
{
    // Parcourir chaque couche
    for (const auto& layerPair : mLayers) {
        const std::vector<Tile>& tiles = layerPair.second;

        // Parcourir chaque tuile de la couche
        for (const auto& tile : tiles) {
            if (tile.getPosition() == position && tile.getIsMineral()) {
                return true;
            }
        }
    }
    return false;
}

bool TilesWorld::isObstacle(const sf::Vector2f& position) const
{
    // Parcourir chaque couche
    for (const auto& layerPair : mLayers) {
        const std::vector<Tile>& tiles = layerPair.second;

        // Parcourir chaque tuile de la couche
        for (const auto& tile : tiles) {
            if (tile.getPosition() == position && tile.getCollidable()) {
                return true;
            }
        }
    }
    return false;
}

bool TilesWorld::isInitialized() const
{
    return !mLayers.empty(); // Remplacez 'mLayers' par le membre qui stocke vos tuiles
}

void TilesWorld::updateTilesForBiome(const Biome& biome, sf::Vector2f playerPosition, float maxDistance) {
    int updatedTiles = 0;

    for (auto& layerPair : mLayers) {
        int layerIndex = layerPair.first;
        std::vector<Tile>& tiles = layerPair.second;

        for (Tile& tile : tiles) {
            if (biome.containsPoint(tile.getPosition().x, tile.getPosition().y)) {

                // Calcul de la distance entre la tuile et le joueur
                float distance = std::sqrt(std::pow(tile.getPosition().x - playerPosition.x, 2) + std::pow(tile.getPosition().y - playerPosition.y, 2));

                // Activer ou désactiver la tuile en fonction de la distance
                if (distance <= maxDistance) {
                    tile.setIsActive(true);
                } else {
                    tile.setIsActive(false);
                }
                
                std::cout << "Player is in this biome. Debugging tiles..." << std::endl;
                std::cout << "Tile Position: (" << tile.getPosition().x << ", " << tile.getPosition().y << ")";
                std::cout << " Tile State: " << (tile.isActive ? "Active" : "Inactive") << std::endl;
                std::cout << "updatedTiles:: " << updatedTiles << std::endl;
            
                updatedTiles++;
            }
        }
    }
}

void TilesWorld::updateAndDrawVisibleTiles(sf::Vector2f playerPosition, float maxDistance) {
    //int updatedTiles = 0;

    // Parcourez toutes les tuiles dans mLayers
    for (auto& layerPair : mLayers) {
        int layerIndex = layerPair.first;
        std::vector<Tile>& tiles = layerPair.second;

        // Parcourez chaque tuile de la couche
        for (Tile& tile : tiles) {
            // Calcul de la distance entre la tuile et le joueur
            float distance = std::sqrt(std::pow(tile.getPosition().x - playerPosition.x, 2) + std::pow(tile.getPosition().y - playerPosition.y, 2));

            // Activer ou désactiver la tuile en fonction de la distance
            if (distance <= maxDistance) {
                tile.setIsActive(true);
                //std::cout << "Tile::setIsActive:: 1 positionX: " << tile.getPosition().x << " positionY:" << tile.getPosition().y << std::endl;
            } else {
                tile.setIsActive(false);
                //s/td::cout << "Tile::setIsActive:: 0 positionX: " << tile.getPosition().x << " positionY:" << tile.getPosition().y << std::endl;
            }

            //updatedTiles++;
        }
    }

    //std::cout << "updatedTiles:: " << updatedTiles << std::endl;
}

void TilesWorld::setResourcesManager(std::shared_ptr<ResourcesManager>& resourcesManager)
{
    mResourcesManager = resourcesManager;
}

void TilesWorld::placeResource(const ResourcesConfig& resourceConfig, int x, int y, int biomeX, int biomeY) {
    if (mResourcesManager) {
        if (isValidPosition(x, y)) { // Vérifiez si la position est valide
            if (isValidResourceConfig(resourceConfig)) { // Vérifiez si la configuration de la ressource est valide

                // Calcul de la position en pixels de la ressource en prenant en compte la taille du biome et la taille de la tuile
                int px = (x + biomeX * biomeMaxSizeX) * tileSize;
                int py = (y + biomeY * biomeMaxSizeY) * tileSize;

                //std::cout << "x: " << x << " y: " << y << " biomeX: " << biomeX << " biomeY: " << biomeY << std::endl;
                //std::cout << "biomeMaxSizeX: " << biomeMaxSizeX << " biomeMaxSizeY: " << biomeMaxSizeY << " tileSize: " << tileSize << " biomeX: " << biomeX << " biomeY: " << biomeY << std::endl;
                //std::cout << "px: " << px << " py: " << py << std::endl;

                mResourcesManager->generateResources(mResourcesType::Ores, resourceConfig.id, px, py);
                //std::cerr << "Position valide.\n";
            }
            else {
                std::cerr << "Configuration de la ressource invalide.\n";
            }
        }
        else {
            std::cerr << "Position invalide.\n";
        }
    }
    else {
        std::cerr << "mResourcesManager est un pointeur null!\n";
    }
}

std::shared_ptr<std::vector<ResourcesConfig>> TilesWorld::getResourcesForBiome(int biomeType) {
    //std::cout << "Obtention des ressources pour le biome: " << biomeType << "\n";

    // Initialisez le shared_ptr une fois au début de la fonction
    std::shared_ptr<std::vector<ResourcesConfig>> resourcesForBiome = std::make_shared<std::vector<ResourcesConfig>>();

    if (mResourcesManager) {
        // Obtenez toutes les configurations de ressources qui peuvent apparaître dans le biome spécifié
        const auto& resourceConfig = mResourcesManager->getResourcesConfig();

        // Parcourez toutes les configurations de ressources et ajoutez celles qui peuvent apparaître dans le biome spécifié
        for (const auto& config : *resourceConfig) {
            if (mResourcesManager->canSpawnInBiome(config, biomeType)) {
                //std::cout << "Resources Config: " << config.id << " Can Spawn in Biome: " << biomeType << " : " << mResourcesManager->canSpawnInBiome(config, biomeType) << std::endl;

                // Ajoutez la configuration de ressource au vector sans créer un nouveau shared_ptr
                resourcesForBiome->push_back(config);
            }
        }
    }

    //std::cout << "Nombre de ressources trouvées: " << resourcesForBiome->size() << std::endl; // Notez l'utilisation de -> pour accéder à la méthode size
    //std::cout << "Nombre de ressources obtenues pour le biome: " << resourcesForBiome->size() << "\n"; // Notez l'utilisation de -> pour accéder à la méthode size
    return resourcesForBiome;
}

bool TilesWorld::shouldGenerateResource(int x, int y) {
    float probability = 0.5f;
    float randomValue = static_cast<float>(rand()) / RAND_MAX;
    //std::cout << "Random Value: " << randomValue << " Probability: " << probability << std::endl;
    return randomValue < probability;
}

int TilesWorld::getRandomResourceIndex(int size) {
    //std::cout << "Obtention d'un index de ressource aléatoire. Taille: " << size << "\n";
    if (size <= 0) return -1; // Retournez -1 si la taille est invalide.

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, size - 1);

    int index = dis(gen);

    //std::cout << "Index de ressource aléatoire obtenu: " << index << "\n";
    return index;
}

bool TilesWorld::isValidPosition(int x, int y) {
    // Vérifiez si la position (x, y) est dans les limites de votre monde
    // Remplacez maxX et maxY par les dimensions de votre monde
    //std::cout << "isValidPosition appelé avec x: " << x << " y: " << y << std::endl;
    bool isValid = false;

    if (x >= 0 && x < mapWidth && y >= 0 && y < mapHeight)
    {
        isValid = true;
    }
    else
    {
        isValid = false;
    }

    //std::cout << "La position est " << (isValid ? "valide" : "invalide") << std::endl;
    return isValid;
}

bool TilesWorld::isValidResourceConfig(const ResourcesConfig& resourceConfig) {
    // Vérifiez d'autres propriétés de resourceConfig si nécessaire
    // Vous devrez définir la logique de validation en fonction de votre implémentation de ResourcesConfig

    // Exemple: vérifiez si l'ID de la ressource est valide
    return resourceConfig.id >= 0 && !resourceConfig.fileSprite.empty();
}

sf::Vector2f TilesWorld::clampPlayerPosition(const sf::Vector2f& playerPosition, const sf::Vector2f& playerSize) {
    sf::Vector2f clampedPosition = playerPosition;
    
    if(clampedPosition.x < 0)
        clampedPosition.x = 0;
    else if(clampedPosition.x > mapWidth * tileSize - playerSize.x)
        clampedPosition.x = mapWidth * tileSize - playerSize.x;

    if(clampedPosition.y < 0)
        clampedPosition.y = 0;
    else if(clampedPosition.y > mapHeight * tileSize - playerSize.y)
        clampedPosition.y = mapHeight * tileSize - playerSize.y;
        
    return clampedPosition;
}

int TilesWorld::calculateDistance(int x1, int y1, int x2, int y2) {
    int distance = std::abs(x2 - x1) + std::abs(y2 - y1);
    return distance;
}

BiomeType TilesWorld::MapToBiome(float value)
{
    // Les valeurs de seuil sont définies pour mapper les valeurs de bruit à des biomes spécifiques.
    // Vous pouvez ajuster ces valeurs de seuil et ajouter/supprimer des biomes selon vos besoins.

    if (value < 0.05) return BiomeType::DEEP_OCEAN; // Océan profond pour les valeurs très basses
    if (value < 0.1) return BiomeType::OCEAN; // Océan pour les valeurs basses
    if (value < 0.15) return BiomeType::RIVER; // Rivière pour les valeurs légèrement plus élevées
    if (value < 0.25) return BiomeType::BEACH; // Plage pour les valeurs proches de la terre
    if (value < 0.35) return BiomeType::PLAINS; // Plaines pour les valeurs moyennes
    if (value < 0.45) return BiomeType::DESERT; // Désert pour les valeurs moyennes à élevées
    if (value < 0.55) return BiomeType::FOREST; // Forêt pour les valeurs élevées
    if (value < 0.7) return BiomeType::HILLS; // Collines pour les valeurs plus élevées
    if (value < 0.85) return BiomeType::MOUNTAIN; // Montagne pour les valeurs très élevées
    if (value < 0.95) return BiomeType::SNOW_MOUNTAIN; // Montagne enneigée pour les valeurs extrêmement élevées

    return BiomeType::POLAR; // Pôle (neige) par défaut si aucune des conditions ci-dessus n'est satisfaite
}

std::vector<std::vector<float>> TilesWorld::GeneratePerlinNoise(int worldSizeX, int worldSizeY, int biomesX, int biomesY) {
    int biomeSizeX = worldSizeX / biomesX;
    int biomeSizeY = worldSizeY / biomesY;
    float scale = static_cast <float> (rand()) / static_cast <float> (5.f);

    noiseMap.resize(worldSizeX, std::vector<float>(worldSizeY));

    assert(worldSizeX > 0 && worldSizeY > 0);
    assert(biomesX > 0 && biomesY > 0);
    assert(biomeSizeX > 0 && biomeSizeY > 0);
    assert(scale > 0);

    for (int y = 0; y < worldSizeY; y++) {
        for (int x = 0; x < worldSizeX; x++) {
            float sampleX = static_cast<float>(x) / biomeSizeX * scale;
            float sampleY = static_cast<float>(y) / biomeSizeY * scale;

            noiseMap[x][y] = (std::sin(sampleX) * std::cos(sampleY) + 1) / 2;
            //std::cout << noiseMap[x][y] << std::endl;
        }
    }
    return noiseMap;
}

/*
Genere des biomes via la fonction "GeneratePerlinNoise"
*/
void TilesWorld::generateBiomes(sf::Vector2f playerPosition, sf::Vector2f playerSize)
{
    //std::cout << "TilesWorld::generateBiomes" << std::endl;

    assert(mapWidth > 0 && mapHeight > 0); // Vérifier que la taille du monde est valide
    assert(biomeSizeX > 0 && biomeSizeY > 0); // Vérifier que le nombre de biomes est valide
    assert(playerPosition.x >= 0 && playerPosition.y >= 0); // Vérifier que la position du joueur est valide
    assert(playerSize.x > 0 && playerSize.y > 0); // Vérifier que la taille du joueur est valide

    GeneratePerlinNoise(mapWidth, mapHeight, biomeSizeX, biomeSizeY);

    int biomeSizeXTemp = mapWidth / biomeSizeX;
    int biomeSizeYTemp = mapHeight / biomeSizeY;

    assert(biomeSizeXTemp > 0 && biomeSizeYTemp > 0); // Vérifier que la taille des biomes est valide

    //std::cout << "biomeSizeX: " << biomeSizeX << " - biomeSizeY: " << biomeSizeY << std::endl;

    for (int x = 0; x < biomeSizeX; ++x) {
        for (int y = 0; y < biomeSizeY; ++y) {
            // Allouer la mémoire directement dans le tableau 2D
            biomes[x][y] = std::make_unique<Biome>();

            float noiseValue = noiseMap[x * biomeSizeXTemp][y * biomeSizeYTemp];

            // Utilisation du pointeur directement depuis le tableau 2D
            biomes[x][y]->type = MapToBiome(noiseValue);
            biomes[x][y]->startX = x * biomeSizeXTemp;
            biomes[x][y]->startY = y * biomeSizeYTemp;
            biomes[x][y]->endX = (x + 1) * biomeSizeXTemp - 1;
            biomes[x][y]->endY = (y + 1) * biomeSizeYTemp - 1;

            biomes[x][y]->sizeX = (biomes[x][y]->endX - biomes[x][y]->startX + 1);
            biomes[x][y]->sizeY = (biomes[x][y]->endY - biomes[x][y]->startY + 1);

            std::cout << "Biome Generated: startX: " << biomes[x][y]->getStartX() << " startY: " << biomes[x][y]->getStartY() << " endX: " << biomes[x][y]->getEndX() << " endY: " << biomes[x][y]->getEndY() << std::endl;

            // Pas besoin de std::move ici, car nous avons déjà alloué la mémoire directement dans le tableau 2D
        }
    }

    updateVisibleMap(playerPosition, playerSize);
}

/*
Met a jour la map visible afin de determiné quelle partie est visible par le joueur
*/
void TilesWorld::updateVisibleMap(sf::Vector2f playerPosition, sf::Vector2f playerSize)
{
    //std::cout << "TilesWorld::updateVisibleMap" << std::endl;

    const int SOME_DISTANCE = 64; // Définir comme constante

    // Clamp player position
    playerPosition = clampPlayerPosition(playerPosition, playerSize);

    // Convertir la position et la taille du joueur en unités de tuiles
    sf::Vector2f playerPosition_in_tiles = playerPosition / static_cast<float>(tileSize);

    // Parcourir la liste des biomes et générer des tuiles et des ressources pour les biomes visibles
    for (int indexX = 0; indexX < biomes.size(); ++indexX) {
        for (int indexY = 0; indexY < biomes[indexX].size(); ++indexY) {
            // Générer le biome s'il n'existe pas encore
            if (biomes[indexX][indexY] == nullptr) {
                biomes[indexX][indexY] = generateBiomeAt(indexX, indexY);
            }

            Biome* biome = biomes[indexX][indexY].get();
            if (!biome || biome->sizeX <= 0 || biome->sizeY <= 0) continue;

            if (!biome->isBiomeGenerated() /* ||canGenerateMoreResources(biome) */ ) {
                int biomeCenterX = (biome->startX + biome->endX) / 2;
                int biomeCenterY = (biome->startY + biome->endY) / 2;
                int distanceToView = calculateDistance(biomeCenterX, biomeCenterY, playerPosition_in_tiles.x, playerPosition_in_tiles.y);

                if (distanceToView <= SOME_DISTANCE) {
                    generateTilesAndResourcesForBiome(biome, indexX, indexY);
                }
            }
        }
    }
}

void TilesWorld::generateTilesAndResourcesForBiome(Biome* biome, int indexX, int indexY) {
    //std::cout << "TilesWorld::generateTilesAndResourcesForBiome" << std::endl;
    
    if (!biome) {
        std::cerr << "Erreur: biome est un pointeur null!" << std::endl;
        return;
    }

    // Vérifier si les tuiles et les ressources ont déjà été générées pour ce biome
    if (!biome->isBiomeGenerated()) {
        // Générer des tuiles pour le biome
        generateTilesForBiome(biome, indexX, indexY);
        // Générer des ressources pour le biome
        //int endX = biome->endX - biome->startX + 1; // Calculer la largeur du biome
        //int endY = biome->endY - biome->startY + 1; // Calculer la hauteur du biome
        int startX = indexX;// *biomeWidth;
        int startY = indexY;// *biomeHeight;
        int endX = startX + biomeWidth - 1; // -1 car les indices commencent à 0
        int endY = startY + biomeHeight - 1; // -1 car les indices commencent à 0
        generateResourcesForBiome(biome, startX, startY, endX, endY);

        // Marquer le biome comme généré
        if (!biome->isBiomeGenerated())
            biome->markAsGenerated();
    }
}

/*
Genere les tuiles afin de créer la map, cette fonction ce base sur "generateBiomes" et "updateVisibleMap"
*/
void TilesWorld::generateTilesForBiome(Biome* biome, int indexX, int indexY) {

    noise.SetNoiseType(FastNoiseLite::NoiseType_Perlin);
    noise.SetFrequency(0.1f);

    //std::cout << "TilesWorld::generateTilesForBiome" << std::endl;

    sf::Image emptyImage;
    //std::cout << "Init::generateTilesForBiome" << std::endl;
    if (!biome) {
        std::cerr << "Biome pointer is null!" << std::endl;
        return;
    }

    if (biome->startX > biome->endX || biome->startY > biome->endY) {
        std::cerr << "Invalid biome start and end values!" << std::endl;
        return;
    }

    std::vector<TilesConfig> possibleTiles;

    for (int layer = 0; layer <= 2; ++layer) {
        for (int x = biome->startX; x <= biome->endX; ++x) {
            if (x < 0 || x >= mapWidth) {
                std::cerr << "x is out of bounds!" << std::endl;
                continue;
            }

            for (int y = biome->startY; y <= biome->endY; ++y) {
                if (y < 0 || y >= mapHeight) {
                    std::cerr << "y is out of bounds!" << std::endl;
                    continue;
                }

                float noiseValue = noise.GetNoise(static_cast<float>(x), static_cast<float>(y));
                

                for (const auto& tile : tilesDB) {
                    if (tile.typeTile != layer) continue;
                    bool canSpawnInBiome = std::find(tile.canSpawnBiome.begin(), tile.canSpawnBiome.end(), biome->type) != tile.canSpawnBiome.end();

                    if (canSpawnInBiome) {
                        // Utilisez noiseValue et rarity pour déterminer si la tuile peut être placée ici
                        float threshold = calculateThreshold(tile, noiseValue); // Vous pouvez définir votre propre logique pour calculer le seuil

                        if (tile.rarity >= threshold) {
                            //std::cout << "threshold:: " << threshold << " tile.rarity -> " << tile.rarity << std::endl;
                            possibleTiles.push_back(tile);
                        }
                    }
                }

                if (!possibleTiles.empty()) {
                    // Sélectionnez et placez une tuile de possibleTiles basée sur la rarity
                    placeTileBasedOnRarity(possibleTiles, x, y, biome);
                }
            }
        }
    }
    possibleTiles.clear();
}

void TilesWorld::generateResourcesForBiome(Biome* biome, int biomeX, int biomeY, int endX, int endY) {
    if (!biome) {
        std::cerr << "Erreur: biome est un pointeur null!" << std::endl;
        return;
    }

    auto possibleResources = getResourcesForBiome(static_cast<int>(biome->type));
    if (!possibleResources) {
        std::cerr << "Erreur: Aucune ressource disponible pour le biome de type " << static_cast<int>(biome->type) << std::endl;
        return;
    }

    float resourceThreshold = 0.7f; // Ajustez ce seuil pour contrôler la densité de ressources

    for (int x = biomeX; x < endX; x++) {
        for (int y = biomeY; y < endY; y++) {
            float noiseValue = noiseMap[x][y]; // Utilisez la valeur de bruit de votre carte de bruit existante
            if (noiseValue > resourceThreshold) { // Si la valeur de bruit est supérieure au seuil, placez une ressource
                int resourceIndex = getRandomResourceIndex(possibleResources->size());
                if (resourceIndex >= 0) {
                    if ((*possibleResources)[resourceIndex].id == 33 || (*possibleResources)[resourceIndex].id == 60) {
                        // std::cout << "Les ressources ne peuvent pas spawn dans l'eau !" << std::endl;
                        continue;
                    }
                    //std::cout << "Resource spawned at position [" << x << ", " << y << "] with ID: " << (*possibleResources)[resourceIndex].id << std::endl;

                    placeResource((*possibleResources)[resourceIndex], x, y, biomeX, biomeY);
                }
            }
        }
    }
}

void TilesWorld::placeTileBasedOnRarity(const std::vector<TilesConfig>& possibleTiles, int x, int y, Biome* biome) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    float totalRarity = 0.0f;
    for (const auto& tile : possibleTiles) {
        totalRarity += tile.rarity;
    }

    float randomValue = dis(gen) * totalRarity;
    float accumulatedRarity = 0.0f;

    for (const auto& tile : possibleTiles) {
        accumulatedRarity += tile.rarity;
        if (randomValue <= accumulatedRarity) {
            //std::cout << "Tile placed at position [" << x << ", " << y << "] with ID: " << tile.id << std::endl;
            createAndPlaceTile(tile, x, y, biome);
            break;
        }
    }
}

float TilesWorld::calculateThreshold(const TilesConfig& tile, float noiseValue) {
    const float ratio = 0.9f;
    float threshold = (noiseValue * tile.rarity) * ratio;
    //std::cout << "Calculated threshold for tile ID " << tile.id << ": " << threshold << std::endl;
    return threshold;
}

void TilesWorld::createAndPlaceTile(const TilesConfig& tileConfig, int x, int y, Biome* biome) {
    //std::cout << "Creating and placing tile at position [" << x << ", " << y << "] with ID: " << tileConfig.id << std::endl;

    // Vérifier si la texture existe
    if (textures.find(tileConfig.id) == textures.end()) {
        std::cerr << "Texture does not exist [id]: " << tileConfig.id << std::endl;
        return;
    }
    sf::Texture& texture = textures[tileConfig.id];

    sf::Image emptyImage;

    // Obtenir la collision mask image
    sf::Image& ImageCollisionMask = (collisionMaskTextures.find(tileConfig.id) != collisionMaskTextures.end())
        ? collisionMaskTextures[tileConfig.id]
        : emptyImage;

    // Créer une nouvelle tuile
    sf::Vector2f position(x, y);
    auto newTile = std::make_shared<Tile>(tileConfig.id, tileConfig.name, texture, tileConfig.sizeX, tileConfig.sizeY, tileConfig.posX, tileConfig.posY, position, tileConfig.collidable, tileConfig.mineral, ImageCollisionMask);

    // Ajouter la tuile au biome et à la couche appropriée
    biome->addTile(x, y, newTile);
    mLayers[tileConfig.typeTile].emplace_back(tileConfig.id, tileConfig.name, texture, tileConfig.sizeX, tileConfig.sizeY, tileConfig.posX, tileConfig.posY, position, tileConfig.collidable, tileConfig.mineral, ImageCollisionMask);
}

std::unique_ptr<Biome> TilesWorld::generateBiomeAt(int x, int y) {
    //std::cout << "Generating biome at position [" << x * biomeSizeX << ", " << y * biomeSizeY << "]" << std::endl;

    auto biome = std::make_unique<Biome>();

    // Calculer la position réelle en fonction de la taille du biome
    int realX = x * biomeSizeX;
    int realY = y * biomeSizeY;

    // Calculer la valeur de bruit à cette position
    float noiseValue = noiseMap[realX][realY];

    // Déterminer le type de biome basé sur la valeur de bruit
    biome->type = MapToBiome(noiseValue);

    // Définir les coordonnées de départ et de fin du biome
    biome->startX = realX;
    biome->startY = realY;
    biome->endX = (x + 1) * biomeSizeX - 1;
    biome->endY = (y + 1) * biomeSizeY - 1;

    // Calculer la taille du biome
    biome->sizeX = (biome->endX - biome->startX + 1);
    biome->sizeY = (biome->endY - biome->startY + 1);

    return biome;
}

//test