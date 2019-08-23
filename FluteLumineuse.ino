#include <FlashStorage.h>
#include <Adafruit_NeoPixel.h>
#include <arduinoFFT.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, 0, NEO_GRB + NEO_KHZ800) ;
arduinoFFT FFT = arduinoFFT();

const bool TRACE = false ;
const size_t DATA_SIZE = 128 ;
const unsigned long PERIOD = 200 ; // us => 5kHz

typedef enum  { MODE_AFFICHAGE_SON, MODE_ACCORDAGE } modeFonctionnement ;

void AdcBooster()
{
  ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |   // Divide Clock by 64.
                   ADC_CTRLB_RESSEL_12BIT;       // Result on 12 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
} // AdcBooster

typedef struct informationsNote
{
  double Frequence ;
  int Note ;
  String Description ;
  unsigned char Rouge ;
  unsigned char Vert ;
  unsigned char Bleu ;
  int MinLevel ;
  int MaxLevel ;
} ;

informationsNote AucuneNote = { 0, 0, "", 0, 0, 0, 0, 0 } ;

informationsNote InformationsNotes[] = {
    {  110.00,  45, "A2" , 255,   0,   0,   500,  5000 }, // 
    {  116.54,  46, "#A2", 255,   0,   0,   500,  5000 }, // 
    {  123.47,  47, "B2" , 255,   0,   0,   500,  5000 }, // 
    {  130.81,  48, "C3" , 255,   0,   0,   500,  5000 }, // 
    {  138.59,  49, "#C3", 255,   0,   0,   500,  5000 }, // 
    {  146.83,  50, "D3" , 255,   0,   0,   500,  5000 }, // 
    {  155.56,  51, "#D3", 255,   0,   0,   500,  5000 }, // 
    {  164.81,  52, "E3" , 255,   0,   0,   500,  5000 }, // 
    {  174.61,  53, "F3" , 255,   0,   0,   500,  5000 }, // 
    {  185.00,  54, "#F3", 255,   0,   0,   500,  5000 }, // 
    {  196.00,  55, "G3" , 255,   0,   0,   500,  5000 }, // 
    {  207.65,  56, "#G3", 255,   0,   0,   500,  5000 }, // 
    {  220.00,  57, "A3" , 255,   0,   0,   500,  5000 }, // 
    {  223.08,  58, "#A3", 255,   0,   0,   500,  5000 }, // 
    {  246.94,  59, "B3" , 255,   0,   0,   500,  5000 }, // -\ 
    {  261.60,  60, "C4" , 255,  20,   0,   500,  6000 }, //   |
    {  277.18,  61, "#C4", 255,  40,   0,   500,  7000 }, //   |
    {  293.67,  62, "D4" , 255,  60,   0,   500,  8000 }, //   |
    {  311.13,  63, "#D4", 255,  80,   0,   500,  9500 }, //   |
    {  329.63,  64, "E4" , 255, 100,   0,   500, 11000 }, //   |
    {  349.23,  65, "F4" , 255, 120,   0,   500, 12500 }, //   |
    {  369.99,  66, "#F4", 255, 140,   0,   500, 14000 }, //   |
    {  392.00,  67, "G4" , 255, 160,   0,   500, 15500 }, //   |
    {  415.30,  68, "#G4", 255, 180,   0,   500, 18000 }, //   |
    {  440.00,  69, "A4" , 255, 200,   0,   500, 19000 }, //   |
    {  466.16,  70, "#A4", 255, 220,   0,   500, 20000 }, //   |
    {  493.88,  71, "B4" , 255, 240,   0,   500, 20000 }, //   |
    {  523.25,  72, "C5" , 255, 255,   0,   500, 20000 }, //   |
    {  554.37,  73, "#C5", 225, 255,   0,   500, 20000 }, //   |
    {  587.33,  74, "D5" , 200, 255,   0,   500, 20000 }, //   |
    {  622.25,  75, "#D5", 175, 255,   0,   500, 20000 }, //   |
    {  659.26,  76, "E5" , 150, 255,   0,   500, 20000 }, //   | Flute Range
    {  698.46,  77, "F5" , 125, 255,   0,   500, 20000 }, //   |
    {  739.99,  78, "#F5", 100, 255,   0,   500, 20000 }, //   |
    {  783.99,  79, "G5" ,  75, 255,   0,   500, 20000 }, //   |
    {  830.61,  80, "#G5",  50, 255,   0,   500, 20000 }, //   |
    {  880.00,  81, "A5" ,  25, 255,   0,   500, 20000 }, //   |
    {  932.33,  82, "#A5",   0, 255,   0,   500, 20000 }, //   |
    {  987.77,  83, "B5" ,   0, 255,  25,   500, 20000 }, //   |
    { 1046.50,  84, "C6" ,   0, 255,  50,   500, 20000 }, //   |
    { 1108.70,  85, "#C6",   0, 255,  75,   500, 20000 }, //   |
    { 1174.70,  86, "D6" ,   0, 255, 100,   500, 20000 }, //   |
    { 1244.50,  87, "#D6",   0, 255, 125,   500, 20000 }, //   |
    { 1318.50,  88, "E6" ,   0, 255, 150,   500, 20000 }, //   |
    { 1396.90,  89, "F6" ,   0, 255, 175,   500, 20000 }, //   |
    { 1480.00,  90, "#F6",   0, 225, 200,   500, 20000 }, //   |
    { 1568.00,  91, "G6" ,   0, 200, 225,   500, 20000 }, //   |
    { 1661.20,  92, "#G6",   0, 150, 255,   500, 20000 }, //   |
    { 1760.00,  93, "A6" ,   0, 100, 255,   500, 20000 }, //   |
    { 1864.70,  94, "#A6",   0,  50, 255,   500, 20000 }, //   |
    { 1975.50,  95, "B6" ,   0,   0, 255,   500, 20000 }, //   |
    { 2093.00,  96, "C7" ,  25,   0, 255,   500, 20000 }, //   |
    { 2217.50,  97, "#C7",  50,   0, 255,   500, 20000 }, //   |
    { 2349.30,  98, "D7" ,  75,   0, 255,   500, 20000 }, // -/ 
    { 2489.00,  99, "#D7", 100,   0, 255,   500, 20000 }, // 
    { 2637.00, 100, "E7" , 100,   0, 225,   500, 20000 }  // 
} ;

typedef struct flashInformationsNote
{
  unsigned char Rouge [sizeof(InformationsNotes)/sizeof(informationsNote)] ;
  unsigned char Vert  [sizeof(InformationsNotes)/sizeof(informationsNote)] ;
  unsigned char Bleu  [sizeof(InformationsNotes)/sizeof(informationsNote)] ;
  int MinLevel        [sizeof(InformationsNotes)/sizeof(informationsNote)] ;
  int MaxLevel        [sizeof(InformationsNotes)/sizeof(informationsNote)] ;
  int InitDone ;
} ;

FlashStorage( ParametrageNotes, flashInformationsNote )

flashInformationsNote InfosNotes ;

int GetNoteIndex( const double FrequenceNoteRecherchee )
{
  for( int i = 1 ; i < (sizeof(InformationsNotes)/sizeof(informationsNote)-2) ; ++i )
  {
    double FreqBasse = (InformationsNotes[i-1].Frequence + InformationsNotes[i].Frequence) / 2 ;
    double FreqHaute = (InformationsNotes[i].Frequence + InformationsNotes[i+1].Frequence) / 2 ;
    if ( ( FrequenceNoteRecherchee >= FreqBasse ) & ( FrequenceNoteRecherchee < FreqHaute ) )
    {
      return i ;
    }
  }
  return -1 ;
}

double Sum(double *vD, uint16_t samples)
{
  double sum = 0.0 ;
  for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
    sum += vD[i] ;
  }
  return sum ;
}

double Moyenne(double *vD, uint16_t samples)
{
  return Sum(vD, samples) / (samples >> 1) ;
}

void LowestPeak(double *vD, uint16_t samples, double samplingFrequency, double *f, double *v)
{
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;
  //If sampling_frequency = 2 * max_frequency in signal,
  //value would be stored at position samples/2
  double moyenne = Moyenne(vD, samples) ;
  for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
    if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
      if (vD[i] > moyenne) {
        IndexOfMaxY = i;
        break ;
      }
    }
  }
  double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
  double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples - 1);
  //double popo =
  if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
    interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
  // returned value: interpolated frequency peak apex
  *f = interpolatedX;
  *v = abs(vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]);
}

class SurveilleDuree
{
  private: unsigned long depart = 0 ;
  private: unsigned long & d ;
  public: SurveilleDuree( unsigned long & duree ) : d(duree) { depart = micros() ; }
  ~SurveilleDuree() { d = micros() - depart ; }
} ;

void StatusLed()
{
  static unsigned short int val = 0 ;
  ++val ;
  if ( val%2 )
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

double vReal[DATA_SIZE];
double vImag[DATA_SIZE];

void LireInformationsFlash()
{
  for ( int i = 0 ; i < sizeof(InformationsNotes)/sizeof(informationsNote) ; ++i )
  {
	InformationsNotes[i].Rouge    = InfosNotes.Rouge   [i] ;
	InformationsNotes[i].Vert     = InfosNotes.Vert    [i] ;
	InformationsNotes[i].Bleu     = InfosNotes.Bleu    [i] ;
	InformationsNotes[i].MinLevel = InfosNotes.MinLevel[i] ;
	InformationsNotes[i].MaxLevel = InfosNotes.MaxLevel[i] ;
  }
}

bool NoteDectectee( struct informationsNote& noteCourante, String note, double amplitude )
{
  if ( amplitude > noteCourante.MinLevel )
  {
    if ( noteCourante.Description == note )
    {
      return true ;
    }
  }
  return false ;
}

int etape = 0 ;
modeFonctionnement FonctionnementEnCours( struct informationsNote& noteCourante, double frequence, double amplitude )
{
  static unsigned long start = 0 ;
  modeFonctionnement m = MODE_AFFICHAGE_SON ;
  static String notes[3] ;
  notes[2] = notes[1] ;
  notes[1] = notes[0] ;  
  if ( amplitude > noteCourante.MinLevel )
  {
    notes[0] = noteCourante.Description ;
  }
  String descriptionNoteDectectee = "" ;
  if ( (notes[0] == notes[1]) & (notes[0] == notes[2]) )
  {
    descriptionNoteDectectee = notes[0] ;
  }
  /*Serial.print("0:") ;
  Serial.print(notes[0]) ;
  Serial.print(" 1:") ;
  Serial.print(notes[1]) ;
  Serial.print(" 2:") ;
  Serial.print(notes[2]) ;
  Serial.print("Descr:") ;
  Serial.println(descriptionNoteDectectee) ;*/
  
  switch( etape )
  {
    case 0 : // Etat indéfini
      if ( descriptionNoteDectectee != "" ) start = millis() ;
      if ( (millis() - start) > 2000 ) ++etape ;
      break ;
    case 1 : // Silence depuis plus de 2 secondes
      if ( descriptionNoteDectectee == "A4" ) ++etape ;
      else if ( !(descriptionNoteDectectee == "") ) etape = 0 ;
      break ;
    case 2 :
      if ( descriptionNoteDectectee == "" ) ++etape ;
      else if ( !(descriptionNoteDectectee == "A4") ) etape = 0 ;
      break ;
    case 3 :
      if ( descriptionNoteDectectee == "A4" ) ++etape ;
      else if ( !(descriptionNoteDectectee == "") ) etape = 0 ;
      break ;
    case 4 :
      if ( descriptionNoteDectectee == "" ) ++etape ;
      else if ( !(descriptionNoteDectectee == "A4") ) etape = 0 ;
      break ;
    case 5 :
      m = MODE_ACCORDAGE ;
      if ( !(descriptionNoteDectectee == "") )
      {
        if ( !(descriptionNoteDectectee == "A4") ) etape = 0 ;
      }
      break ;
  }
  return m ;
}

void DoAffichageSon( struct informationsNote& noteCourante, double frequence, double amplitude )
{
  strip.setPixelColor( 0, 0, 0, 0 ) ;
  if ( &noteCourante != &AucuneNote )
  {
    if ( amplitude > noteCourante.MinLevel )
    {
      double BrightnessRatio = ((double)amplitude) / noteCourante.MaxLevel ;
      if ( BrightnessRatio > 1 ) BrightnessRatio = 1 ;
      strip.setPixelColor( 0,
        (unsigned char)((double)(noteCourante.Rouge)*BrightnessRatio),
        (unsigned char)((double)(noteCourante.Vert )*BrightnessRatio) ,
        (unsigned char)((double)(noteCourante.Bleu )*BrightnessRatio) ) ;
    } 
  }
  strip.show() ;
}

void DoAccordage( struct informationsNote& noteCourante, double frequence, double amplitude )
{
  strip.setPixelColor( 0, 0, 0, 0 ) ;
  if ( &noteCourante != &AucuneNote )
  {
    if ( amplitude > noteCourante.MinLevel )
    {
      if      ( frequence > noteCourante.Frequence * 1.010 ) { strip.setPixelColor( 0,   0,   0, 255 ) ; }
      else if ( frequence > noteCourante.Frequence * 1.002 ) { strip.setPixelColor( 0,   0, 128, 192 ) ; }
      else if ( frequence < noteCourante.Frequence * 0.990 ) { strip.setPixelColor( 0, 255,   0,   0 ) ; }
      else if ( frequence < noteCourante.Frequence * 0.998 ) { strip.setPixelColor( 0, 192, 128,   0 ) ; }
      else                                                   { strip.setPixelColor( 0,   0, 255,   0 ) ; }
    }
  }
  strip.show() ;
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  analogReadResolution(12) ;
  if ( TRACE )
  {
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println("Démarrage...") ;
  }
  
  AdcBooster() ;

  ParametrageNotes.read( &InfosNotes ) ;
  if ( InfosNotes.InitDone != 0 )
  {
    LireInformationsFlash() ;
    if ( TRACE )
    {
      Serial.println("Paramétrage flash chargé.") ;
    }
  }
  else
  {
    if ( TRACE )
    {
      Serial.println("Paramétrage par défaut.") ;
    }
  }
  
  strip.begin();
  strip.setBrightness(50);
  strip.show(); // Initialize all pixels to 'off'
}


void loop() {

  int i ;
  unsigned long DureeAcquitision = 0 ;
  { SurveilleDuree s( DureeAcquitision ) ;
    unsigned long debut = micros() ;
    for( i = 0 ; i < DATA_SIZE ; ++i )
    {
      vReal[i] = analogRead(A0) ;
      vImag[i] = 0 ;
      while( (micros() - debut) < PERIOD ) {}
      debut += PERIOD ;
    }
  }

  double frequence;
  double amplitude;

  unsigned long DureeCalcul = 0 ;
  { SurveilleDuree s( DureeCalcul ) ;
    double f2 ;
    double a2 ;
    FFT.Windowing(vReal, DATA_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD) ;
    FFT.Compute(vReal, vImag, DATA_SIZE, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, DATA_SIZE);
    FFT.MajorPeak(vReal, DATA_SIZE, 1000000/PERIOD, &f2, &a2);
    LowestPeak(vReal, DATA_SIZE, 1000000/PERIOD, &frequence, &amplitude);
    // Si le plus grand pic est au double du premier, et qu'il est bien plus gros, on le prend à la place.
    if  ( ( f2 < frequence * 1.9 ) & ( f2 < frequence * 2.1 ) )
    {
      if ( a2 > amplitude * 2 )
      {
        frequence = f2 ;
        amplitude = a2 ;
      }
    }
  }
  
  int index ;
  index = GetNoteIndex(frequence) ;
  
  if ( TRACE )
  {
    Serial.print(frequence, 6);
    Serial.print("\t");
    if ( index >= 0 ) if ( amplitude > InformationsNotes[index].MinLevel ) Serial.print(InformationsNotes[index].Description);
    Serial.print("\t");
    Serial.print(amplitude, 6) ;
    Serial.print("\tAcq : ") ;
    Serial.print(DureeAcquitision) ;
    Serial.print(" us, Calc : " ) ;
    Serial.print(DureeCalcul) ;
    Serial.print(" us\t" ) ;
    Serial.print(etape) ;
    Serial.println("." ) ;
  }
 
  informationsNote& InformationsNoteCourante = index >= 0 ? InformationsNotes[index] : AucuneNote ;
  
  switch( FonctionnementEnCours( InformationsNoteCourante, frequence, amplitude ) )
  {
		case MODE_AFFICHAGE_SON :
			DoAffichageSon( InformationsNoteCourante, frequence, amplitude ) ;
		break ;
		case MODE_ACCORDAGE :
			DoAccordage( InformationsNoteCourante, frequence, amplitude ) ;
		break ;
  }
  
  StatusLed() ;
}
