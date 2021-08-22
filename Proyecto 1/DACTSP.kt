import kotlin.Double.Companion.POSITIVE_INFINITY
import java.io.File
import kotlin.math.*
import kotlin.system.exitProcess

typealias Point = Pair<Double, Double>
typealias Rectangle = Pair<Point, Point>
typealias CityArray = Array<Point>
typealias Cycle = Array<Pair<Point, Point>>

// ----------- PRINCIPAL ----------

/**
 * Función principal que recibe como entrada los argumentos
 * de la entrada estándar indicando una ruta válida a un 
 * archivo de entrada en formato TSPLIB, y una ruta a un
 * archivo de salida.
 *
 * Ejecuta el algoritmo de divideAndConquerTSP sobre las ciudades
 * contenidas en el archivo de entrada y traslada el ciclo de retorno
 * de forma que comience con la primera ciudad en la entrada.
 *
 * Imprime el nomnbre del archivo de entrada dado y la distancia
 * total del tour obtenido por la salida estándar, y escribe en
 * el archivo de salida indicado los resultados en formato TSPLIB.
 */
fun main(args: Array<String>) {
    val P = extraerDatos(args[0])
    val PCopy = P.copyOf()

    var tour = divideAndConquerTSP(P)

    // Arregla el ciclo para que comience en la primera ciudad
    for ((i, lado) in tour.withIndex()) {
        if (lado.first == PCopy[0] || lado.second == PCopy[0]) {
        	// El tour ya estaba ordenado
        	if (i == 0 && (tour[1].first != PCopy[0] && tour[1].second != PCopy[0])) {
        		break
        	} else {
	            tour = Cycle (tour.size, { tour[(it + i + 1) % tour.size] } )
	            break
        	}
	        	
        }
    }

    val dist = calcularDistanciaTour(tour)
    println("TSP instance: ${args[0]}\n")
    println("Total distance: $dist\n")
    escribirTSP(args[1], PCopy, tour, dist)
    if (!tourValido(P, tour, PCopy[0])) exitProcess(1)
}

// ----------- ALGORITMO ----------

/**
 * Entrada: Un arreglo P con coordenadas de ciudades
 * Salida: Un ciclo C solución válida del TSP.
 */
fun divideAndConquerTSP(P: CityArray): Cycle {
    val n = P.size

    when (n) {
        0 -> return arrayOf()
        1 -> return cicloUnaCiudad(P)
        2 -> return cicloDosCiudades(P)
        3 -> return cicloTresCiudades(P)
        else -> {
            val (pderecha, pizquierda) = obtenerParticiones(P)
            val c1 = divideAndConquerTSP(pderecha) 
            val c2 = divideAndConquerTSP(pizquierda)

            return combinarCiclos(c1, c2)
        }
    }
}

/**
 * Entrada: Un arreglo P con coordenadas de ciudades.
 * Salida: Un par con las coodenadas inferior izquierda
 *         y superior derecha del rectángulo que contiene
 *         todos los puntos en P.
 */
fun obtenerRectangulo(P: CityArray): Rectangle {
    var (x1, y1) = Pair(P[0].first, P[0].second)
    var (x2, y2) = Pair(P[0].first, P[0].second)

    // Encuentra (minX, minY) y (maxX, maxY)
    for (punto in P) {
        if (punto.first < x1) x1 = punto.first
        if (punto.second < y1) y1 = punto.second

        if (punto.first > x2) x2 = punto.first
        if (punto.second > y2) y2 = punto.second 
    }

    return Pair(Pair(x1, y1), Pair(x2, y2))
}

/**
 * Entrada: Un par con las coordenadas inferior izquierda
 *          y superior derecha de un rectámgulo.
 * Salida: Un par con las dimensiones de los lados paralelos
 *         al eje x y al eje y, respectivamente.
 */
fun obtenerRectanguloDims(rectangulo: Rectangle): Point {
    val (p1, p2) = rectangulo
    return Pair(p2.first - p1.first, p2.second - p1.second ) 
}

/**
 * Entrada: Dos arreglos P1 y P2 con coordenada de ciudades.
 * Salida: - true si uno de los arreglos está vacio y el otro
 *           tiene más de 3 elementos
 *         - false en otro caso.
 */
fun estanDesbalanceadas(P1: CityArray, P2: CityArray): Boolean =
    (P1.size == 0 && P2.size > 2) || (P1.size > 2 && P2.size == 0)

/** Retorna 'Y' si [eje] = 'X', si no, retorna 'X' */
fun cambiarEje(eje: Char): Char = if (eje == 'X') 'Y' else 'X'

/**
 * Entrada: Un arreglo P con coordenadas de ciudades.
 *          Un caracter con el ejeDeCorte ('X' o 'Y').
 * Salida: La ciudad que resulta aproximadamente en el medio
 *         al ordenar P por el respectivo ejeDeCorte.
 */
fun obtenerPuntoDeCorte(P: CityArray, ejeDeCorte: Char): Point {
    val n = P.size
    val pos = if (n % 2 == 1) n / 2 else n / 2 - 1

    ordenarPares(P, ejeDeCorte)

    return P[pos]
}

/**
 * Entrada: Un par con las coordenadas inferior izquierda
 *          y superior derecha de un rectángulo.
 *          Un caracter con un eje ('X' o 'Y').
 * Salida: Las coordenadas del punto medio del rectángulo.
 */
fun obtenerPuntoDeCorteMitad(rectangulo: Rectangle, eje: Char): Point {

    val (p1, _) = rectangulo
    val (xMin, yMin) = Pair(p1.first, p1.second)
    val (xDim, yDim) = obtenerRectanguloDims(rectangulo)

    return when (eje) {
        'X' -> Pair(xMin + xDim / 2, yMin)
        else -> Pair(xMin,  yMin + yDim / 2)
    }
}

/**
 * Entrada: Un arreglo P con coordenadas de ciudades.
 *          Un caracter con el ejeDeCorte ('X' o 'Y').
 *          Un par de Double con el puntoDeCorte.
 *          Un par de pares con las coordenadas de las esquinas inferior
 *          izquierda y superior derecha de un rectángulo.
 * Salida: Un par de rectángulos resultantes de trazar una recta
 *         perpendicular al ejeDeCorte, excluyendo del segundo el
 *         espacio vacío.
 */
fun aplicarCorte(
    ejeDeCorte: Char,
    puntoDeCorte: Point, 
    rectangulo: Rectangle
    ): Pair<Rectangle, Rectangle> {

    val (p1, p4) = rectangulo

    return when (ejeDeCorte) {
        'X' -> {
            val p2 = Pair(puntoDeCorte.first, p4.second)
            val p3 = Pair(puntoDeCorte.first + 0.0001, p1.second)
            Pair(Pair(p1, p2), Pair(p3, p4))
        }

        else -> {
            val p2 = Pair(p4.first, puntoDeCorte.second)
            val p3 = Pair(p1.first, puntoDeCorte.second + 0.0001)
            Pair(Pair(p1, p2), Pair(p3, p4))
        }
    }
}

/**
 * Entrada: Una par de enteros que representa las coordenadas de una
 *          [ciudad].
 *          Un par con las coordenadas superior izquierda e inferior
 *          derecha de un rectangulo.
 * Salida: - true si la ciudad se encuentra dentro del área delimitada
 *           por el rectángulo.
 *         - false de otra forma.
 */
fun estaEnRectangulo(ciudad: Point, rectangulo: Rectangle): Boolean {
    val (p1, p2) = rectangulo
    val (xMin, yMin) = p1
    val (xMax, yMax) = p2

    return if (xMin <= ciudad.first && ciudad.first <= xMax) {
        yMin <= ciudad.second && ciudad.second <= yMax
        } else {
            false
        }
}

/**
 * Entrada: Un arreglo ordenado P con coordenadas de ciudades.
 *          Un par con las coordenadas superior izquierda e inferior
 *          derecha de un rectangulo.
 * Salida: Un arreglo con todos los puntos en P dentro del area
 *         delimitada por el rectángulo.
 */
fun obtenerPuntosRectangulo(P: CityArray, rectangulo: Rectangle): CityArray {
    val n = P.size

    // Se busca el rango [a,b) de las ciudades dentro del rectángulo
    var a = n
    var b = n

    /* a = indice de la primera ciudad en P dentro del
    rectangulo (desde el primero) */
    for ((i, ciudad) in P.withIndex()) {
        if (estaEnRectangulo(ciudad, rectangulo)) {
            a = i
            break
        }
    }

    /* b = indice de la primera ciudad en P dentro del
    rectangulo (desde el último) + 1 */
    for (i in (n - 1) downTo 0) {
        if (estaEnRectangulo(P[i], rectangulo)) {
            b = i + 1
            break
        }
    }

    return CityArray(b - a, { P[it + a] })
}

/**
 * Entrada: Un arreglo P con coordenada de ciudades.
 * Salida: Un par con dos arreglos de coordenadas de ciudades. 
 */
fun obtenerParticiones(P: CityArray): Pair<CityArray, CityArray> {

    val rectangulo = obtenerRectangulo(P)
    val (xDim, yDim) = obtenerRectanguloDims(rectangulo)

    var ejeDeCorte = if (xDim > yDim) 'X' else 'Y'

    var puntoDeCorte = obtenerPuntoDeCorte(P, ejeDeCorte)
    var rectangulos = aplicarCorte(ejeDeCorte, puntoDeCorte, rectangulo)
    var rectanguloIzq = rectangulos.first
    var rectanguloDer = rectangulos.second

    var particionIzq = obtenerPuntosRectangulo(P, rectanguloIzq)
    var particionDer = obtenerPuntosRectangulo(P, rectanguloDer)

    if (estanDesbalanceadas(particionIzq, particionDer)) {
        ejeDeCorte = cambiarEje(ejeDeCorte)
        
        puntoDeCorte = obtenerPuntoDeCorte(P, ejeDeCorte)
        rectangulos = aplicarCorte(ejeDeCorte, puntoDeCorte, rectangulo)
        rectanguloIzq = rectangulos.first
        rectanguloDer = rectangulos.second

        particionIzq = obtenerPuntosRectangulo(P, rectanguloIzq)
        particionDer = obtenerPuntosRectangulo(P, rectanguloDer)

        if (estanDesbalanceadas(particionIzq, particionDer)) {
            puntoDeCorte = obtenerPuntoDeCorteMitad(rectangulo, ejeDeCorte)
            rectangulos = aplicarCorte(ejeDeCorte, puntoDeCorte, rectangulo)
            rectanguloIzq = rectangulos.first
            rectanguloDer = rectangulos.second

            particionIzq = obtenerPuntosRectangulo(P, rectanguloIzq)
            particionDer = obtenerPuntosRectangulo(P, rectanguloDer)
        }
    }
    
    return Pair(particionIzq, particionDer)
}

/**
 * Entrada: Dos coordenadas a y b.
 * Salida: Un entero con la distancia euclideana entre a y b.
 * 
 * Los cálculos son hechos usando aritmética de doble precisión,
 * el resultado es redondeado al entero más cercano.
 */
fun distancia(a: Point, b: Point): Double {
    val (x1, y1) = a
    val (x2, y2) = b

    val xd = x1 - x2
    val yd = y1 - y2

    val dist = sqrt(xd * xd + yd * yd)

    return dist
}

/**
 * Entrada: Dos coordenadas a y b.
 * Salida: Un entero con la distancia euclideana entre a y b.
 * 
 * Los cálculos son hechos usando aritmética de doble precisión,
 * el resultado es redondeado al entero más cercano.
 */
fun distanciaEntera(a: Point, b: Point): Int {
    val (x1, y1) = a
    val (x2, y2) = b

    val xd = x1 - x2
    val yd = y1 - y2

    val dist = sqrt(xd * xd + yd * yd) + 0.5

    return dist.toInt()
}

fun distanciaGanada(dOLD1: Double, dOLD2: Double, dNEW1: Double, dNEW2: Double): Double =
    (dNEW1 + dNEW2) - (dOLD1 + dOLD2)

/**
 * Ordena una secuencia de lados de tal forma que cada
 * elemento está enlazado con sus adyacentes. Usa una
 * variación de Selection Sort.
 *
 * Entrada: Una secuencia de lados C que conforma un
 *          ciclo válido.
 */
fun construirTour(C: Cycle) {
    val n = C.size

    for (i in 1 until (n - 1)) {
        var indice = i
        var (p1, p2) = C[i - 1]

        // Halla el lado que enlaza con el anterior
        for (j in i until n) {
            val enlace = arrayOf(C[j].first, C[j].second)

            if (p1 in enlace || p2 in enlace) {
                indice = j
                break
            }
        }
        // Intercambia C[i] y C[indice]
        val temp = C[i]
        C[i] = C[indice]
        C[indice] = temp
    }
}

/**
 * Entrada: Dos secuencias de lados ciclo1 y ciclo2 que
 *          representan dos tours de ciudades.
 * Salida: Un tour ciclo3 que es la unión de ciclo1 y ciclo2.
 */
fun combinarCiclos(ciclo1: Cycle, ciclo2: Cycle): Cycle {
    val n = ciclo1.size
    val m = ciclo2.size

    if (n == 0) return ciclo2
    if (m == 0) return ciclo1

    var ladosAgregarC1 = Pair(Pair(0.0, 0.0), Pair(0.0, 0.0))
    var ladosAgregarC2 = Pair(Pair(0.0, 0.0), Pair(0.0, 0.0))
    var ladosEliminarC1 = Pair(Pair(0.0, 0.0), Pair(0.0, 0.0))
    var ladosEliminarC2 = Pair(Pair(0.0, 0.0), Pair(0.0, 0.0))

    var minG = POSITIVE_INFINITY
    for (ladoCiclo1 in ciclo1) {
        val (a, b) = ladoCiclo1
        val dOLD1 = distancia(a, b)

        for (ladoCiclo2 in ciclo2) {
            val (c, d) = ladoCiclo2
            val dOLD2 = distancia(c, d)

            val dNEW1 = distancia(a, c)
            val dNEW2 = distancia(b, d)
            val dNEW3 = distancia(a, d)
            val dNEW4 = distancia(b, c)

            val g1 = distanciaGanada(dOLD1, dOLD2, dNEW1, dNEW2)
            val g2 = distanciaGanada(dOLD1, dOLD2, dNEW3, dNEW4)

            val ganancia = minOf(g1, g2)
            if (ganancia <= minG) {
                minG = ganancia

                if (g1 < g2) {
                    ladosAgregarC1 = Pair(a, c)
                    ladosAgregarC2 = Pair(b, d)
                } else {
                    ladosAgregarC1 = Pair(a, d)
                    ladosAgregarC2 = Pair(b, c)
                }

                ladosEliminarC1 = Pair(a, b)
                ladosEliminarC2 = Pair(c, d)
            }
        }
    }

    /* Crea el ciclo 3 como la concatenación de ciclo1 y ciclo2.
    Reemplaza los lados a eliminar por los lados a agregar */
    val ciclo3 = Cycle(m + n, {
        if (it < n) {
            if (ciclo1[it] == ladosEliminarC1) ladosAgregarC1
            else ciclo1[it]
        } else {
            if (ciclo2[it - n] == ladosEliminarC2) ladosAgregarC2
            else ciclo2[it - n]
        }
    })

    construirTour(ciclo3)

    return ciclo3

}

// --------- ORDENAMIENTO Y BÚSQUEDA ----------

/**
 * Entrada: Un arreglo P ordenado de coordenadas de ciudades.
 * Salida: Un caracter que indica el eje respecto al cual P
 *         está ordenado ('X' o 'Y').
 */
fun ejeDeOrden(P: CityArray): Char{
    val n = P.lastIndex

    for (i in 0 until n) {
        if (P[i].first > P[i + 1].first) return 'Y'
        if (P[i].second > P[i + 1].second) return 'X'
    }
    return 'X'
}

/**
 * Entrada: Un arreglo P ordenado de coordenadas de ciudades.
            Un par de Double que representa una ciudad.
            Un caracter con el [eje] ordenado de P ('X' o 'Y').
 * Salida: Un entero i tal que P[i] = ciudad.
 *         -1 si no está en el arreglo. 
 */
fun busquedaBinaria(P: CityArray, ciudad: Point, eje: Char): Int {
    var N = P.size
    var (izq, der) = Pair(0, N - 1)

    while (izq <= der) {
        val medio = (izq + der) / 2

        if (P[medio] == ciudad) return medio
        else if (menorQue(ciudad, P[medio], eje)) der = medio - 1
        else izq = medio + 1
    }

    return -1
}

/**
 * Intercambia los elementos i y j de P, un arreglo de coordenadas de
 * ciudades.
 */
fun intercambiar(P: CityArray, i: Int, j: Int) {
    val temp = P[i]
    P[i] = P[j]
    P[j] = temp
}

/** Retorna el mayor entero menor o igual que lg(n). */
fun floorLg(n: Int): Int = floor(log2(n.toDouble())).toInt()

/**
 * Entrada: Dos pares p1 y p2 de Double.
 *          Un caracter con un eje ('X' o 'Y')
 * Salida: - true si p1 < p2 con respecto a eje, o si
 *           p1 = p2 con respecto a eje y p1 < p2 con
 *           respecto al eje opuesto.
 *         - false de otra forma.
 */
fun menorQue(p1: Point, p2: Point, eje: Char): Boolean {
    val (x1, y1) = p1
    val (x2, y2) = p2

    return when (eje) {
        'X' -> {
            if ((x1 < x2) || (x1 == x2 && y1 < y2)) {
                true
            } else {
                false
            }
        }

        else -> {
            if ((y1 < y2) || (y1 == y2 && x1 < x2)) {
                true
            } else {
                false
            }
        }
    }
}

/** 
 * Ordena un arreglo con coordenada de ciudades P en orden
 * no decreciente usando Insertion Sort.
 * Las comparaciones se hacen con respecto al [eje].
 */
fun insertar(P: CityArray, eje: Char = 'X') {
    val n = P.size

    for (i in 1 until n) {
        var j = i
        while (j != 0 && menorQue(P[j], P[j - 1], eje)) {
            intercambiar(P, j, j - 1)
            j--
        }
    }
}

/**
 * Mantiene las propiedades de un Max-Heap en un arreglo P con
 * coordenadas de ciudades en el rango [start, end), con nodo 
 * padre en el indice i.
 * Las comparaciones se hacen con respecto al [eje].
 */
fun maxHeapify(P: CityArray, i: Int, start: Int, end: Int, eje: Char = 'X') {
    var (l, r) = Pair(2 * i + 1 - start, 2 * i + 2 - start)
    var largest: Int

    largest = if (l < end && menorQue(P[i], P[l], eje)) l else i
    largest = if (r < end && menorQue(P[largest], P[r], eje)) r else largest 

    if (largest != i) {
        intercambiar(P, i, largest)
        maxHeapify(P, largest, start, end, eje)
    }
}

/**
 * Construye un Max-Heap con todos los elementos en el rango [a,b) 
 * de un arreglo P con coordenadas de ciudades.
 * Las comparaciones se hacen con respecto al [eje].
 */
fun construirMaxHeap(P: CityArray, a: Int, b: Int, eje: Char = 'X') {
    val n = b - a
    for (i in (n / 2 - 1 + a)  downTo a) {
        maxHeapify(P, i, a, b, eje)
    }
}

/**
 * Ordena un arreglo P con coordenadas de ciudades en el rango [a, b)
 * en orden no decreciente usando Heap Sort.
 * Las comparaciones se hacen con respecto al [eje].
 */
fun heapSort(P: CityArray, a: Int, b: Int, eje: Char = 'X') {
    construirMaxHeap(P, a, b, eje)

    var end = b
    for (i in (b - 1) downTo (a + 1)) {
        intercambiar(P, i, a)
        end--
        maxHeapify(P, a, a, end, eje)
    }
}

/**
 * Reordena los elementos en P formando una partición.
 *
 * Entrada: Un arreglo P con con coordenadas de ciudades.
 *          Dos enteros a, b que conforman el rango [a..b) reordenar.
 *          Un caracter con el eje ('X' o 'Y').
 * Salida: Un entero l tal que P[k] <= x para todo a <= k <= l and
 *         P[k] >= x para todo l < k < b.
 *
 * Las comparaciones se hacen con respecto al [eje].
 */
fun particion(P: CityArray, a: Int, b: Int, eje: Char = 'X'): Int {
    var (i, j) = Pair(a - 1, b)
    val x = P[b - 1]

    while (true) {
        do { j-- } while (menorQue(x, P[j], eje))
        do { i++ } while (menorQue(P[i], x, eje))

        if (i < j) intercambiar(P, i, j) else return i
    }
}

/**
 * Ordena un arreglo P con coordenadas de ciudades sobre el
 * rango [a, b) usando Quicksort con el algoritmo de
 * particionaminento de Hoare. Limita la cantidad de llamadas
 * recursivas a [profundidadMaxima] y ordena con heapSort cuando
 * se supera el límite.
 * Las comparaciones se hacen con respecto al [eje].
 */
fun qsortCiclo(P: CityArray, a: Int, b: Int, profundidadMaxima: Int, eje: Char = 'X') {
    var r = b

    while (r - a > 16) {
        if (profundidadMaxima == 0) {
            heapSort(P, a, r, eje)
            return
        }

        val p = particion(P, a, r, eje)
        qsortCiclo(P, p, r, profundidadMaxima - 1, eje)
        r = p
    }
}

/**
 * Ordena un arreglo P con coordenadas de ciudades en orden
 * no decreciente usando Introsort.
 * Las comparaciones son hechas con respecto al [eje].
 */
fun ordenarPares(P: CityArray, eje: Char = 'X') {
    val n = P.size
    qsortCiclo(P, 0, n, 2 * floorLg(n), eje)
    insertar(P, eje)
}

// ----- CICLOS -----

/** Devuelve un ciclo trivial con la única ciudad en P, un arreglo con una sola ciudad. */
fun cicloUnaCiudad(P: CityArray): Cycle = arrayOf(Pair(P[0], P[0]))

/** Devuelve un ciclo trivial con las dos ciudades de P, un arreglo con dos ciudades. */
fun cicloDosCiudades(P: CityArray): Cycle = arrayOf(Pair(P[0], P[1]), Pair(P[1], P[0]))

/** Devuelve un ciclo trivial con las tres ciudades P, un arreglo con tres ciudades. */
fun cicloTresCiudades(P: CityArray): Cycle = 
    arrayOf(Pair(P[0], P[1]), Pair(P[1], P[2]), Pair(P[2], P[0]))

// ----- LECTURA Y ESCRITURA DE ARCHIVOS -----

/**
 * Entrada: Una string [path] con un camino válido a un archivo
 *          de la librería TSPLIB.
 * Salida: Un arreglo con las coordenadas de las ciudades del
 *         archivo en [path]. 
 */
fun extraerDatos(path: String): CityArray {

    val contenido = File(path).readLines().toTypedArray()

    val size = contenido[3]
        .split(" ")
        .filter({ it.toIntOrNull() != null })[0]
        .toInt()

    return CityArray(size, {
        val column = (contenido[it + 6]
            .split(" ")
            .filter { it != "" })
        Pair(column[1].toDouble(), column[2].toDouble())
        })
}

/**
 * Escribe los datos de la salida del algoritmo TSP en el formato
 * especificado.
 *
 * Entrada: Una string [path] con un camino válido al archivo que
 *          se creará o sobreescribirá con los datos de salida,
 *          Un arreglo P con coordenadas de ciudades que representan
 *          la entrada de divideAndConquerTSP.
 *          Un ciclo C salida de divideAndConquerTSP, previamente 
 *          desplazado para que comience y termine con la primera
 *          ciudad en P.
 *          Un entero dist con la distancia de C, calculada según
 *          indica la documentación de TSPLIB.      
 */
fun escribirTSP(
    path: String,
    P: CityArray, C: Cycle,
    dist: Int
    ) {
    val archivo = File(path)

    val nameSplit = path.split("/")
    val name = nameSplit[nameSplit.lastIndex]
    archivo.writeText("NAME : $name\n")

    val escribir = arrayOf("COMMENT : Length $dist\n", "TYPE : TOUR\n",
        "DIMENSION : ${P.size}\n", "TOUR_SECTION\n", "1\n")

    for (linea in escribir) archivo.appendText(linea)

    var actual = P[0]

    for (lado in C) {
        actual = if (lado.first == actual) lado.second else lado.first
        if (actual != P[0]) {
            for ((i, ciudad) in P.withIndex()) {
                if (ciudad == actual) {
                    archivo.appendText("${i + 1}\n")
                    break
                }
            }
        }
    }
    archivo.appendText("-1\n")
    archivo.appendText("EOF\n")
}

// ------ FUNCIIONES DE AYUDA Y DEPURACIÓN --------

/** Imprime un arreglo de pares de coordenadas C en la salida estándar. */
fun imprimirCiclo(C: Cycle) {
    C.forEach { print("$it ") }
    println()
}

/** 
 * Entrada: Un arreglo C de pares de coordenadas.
 * Salida: Un entero con la suma de las distancias entre cada par de
 *         coordenadas de C. 
 */
fun calcularDistanciaTour(C: Cycle): Int {
    var suma = 0
    for (lado in C) suma += distanciaEntera(lado.first, lado.second)

    return suma
}

/** 
 * Efectúa todas las verificaciones correspondientes a la correctitud
 * de un ciclo salida del algoritmo TSP:
 *
 *  - El tamaño del ciclo debe ser igual al número de ciudades.
 *  - El ciclo comienza con la primera ciudad de la entrada del algoritmo.
 *  - El ciclo termina con la primera ciudad de la entrada del algoritmo.
 *  - Toda ciudad en el ciclo está en el arreglo de la entrada del algoritmo.
 *  - Toda ciudad en el arreglo de entrada del algoritmo aparece exactamente
 *    en dos lados del ciclo.
 *  - Todo par de coordenadas en el ciclo enlaza con su adyacente.
 *
 * Entrada: Un arreglo P con coordenadas de ciudades (la entrada original del 
 *          algoritmo TSP).
 *          Un arreglo C con pares de coordenadas que representa un tour.
 *          Las coordenadas de la [primeraCiudad] de la entrada original del
 *          algoritmo TSP.
 * Salida: - true si C cumple con todas las condiciones de un ciclo valido
 *           solución del algoritmo TSP aplicado a la instancia P.
 *         - false de otra forma.
 */
fun tourValido(P: CityArray, C: Cycle, primeraCiudad: Point): Boolean {

    val n = P.size
    val m = C.size

    if (n != m) {
        println("El tamaño del ciclo es distinto al número de ciudades")
        return false
    }

    if (C[0].first != primeraCiudad && C[0].second != primeraCiudad) {
        println("El ciclo no comienza con la primera ciudad en P.")
        return false
    }

    if (C[m - 1].first != primeraCiudad && C[m - 1].second != primeraCiudad) {
        println("El ciclo no termina con la primera ciudad en P.")
        return false
    }

    val eje = ejeDeOrden(P)
    val frecCiudades = IntArray(n)
    var contador = 0

    for ((k, lado) in C.withIndex()) {
        val (a, b) = lado
        val (c, d) = C[(k + 1) % m]

        val i = busquedaBinaria(P, a, eje)
        val j = busquedaBinaria(P, b, eje)

        if (i == -1 || j == -1) {
            println("El tour contiene una ciudad que no está en el arreglo de ciudades")
            return false
        }

        /* Suma 1 a la frecuencia por cada vez que la ciudad aparezca
        más de dos veces en algún par del tour. */
        if (++frecCiudades[i] > 1) contador++
        if (++frecCiudades[j] > 1) contador++

        if (a != c && a != d && b != c && b != d) {
            println("El tour contiene un lado que no enlaza con su adyacente")
            return false
        }
    }

    if (contador != n) {
        if (contador > n) {
            println("El tour contiene una ciudad que es visitada más de una vez")
        } else {
            println("El tour contiene algún lado abierto")
        }
        
        return false
    }

    return true
}