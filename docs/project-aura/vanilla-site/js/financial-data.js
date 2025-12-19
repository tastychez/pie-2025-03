// Financial breakdown data
const financialData = {
  categories: [
    {
      category: "Electronics",
      data: [
        {
          name: "50 amp breaker (30 amp continous and 80 amp surge + 4 amp constant)",
          cost: 12,
          quantity: 1,
          total: 12,
        },
        { name: "Anderson Connector", cost: 22.5, quantity: 1, total: 22.5 },
        {
          name: "Big Power Distribution Panel",
          cost: 31,
          quantity: 1,
          total: 32,
        },
      ],
    },
    {
      category: "Drivetrain",
      data: [
        { name: "Battery", cost: 138.11, quantity: 1, total: 138.11 },
        {
          name: "MY1016 24V 250w DC motor with Gearbox",
          cost: 50.29,
          quantity: 2,
          total: 100.58,
        },
        { name: "5/8 Shaft Collars x8", cost: 6.88, quantity: 2, total: 13.76 },
        {
          name: "5/8 Flanged Ball Bearing x10",
          cost: 15.93,
          quantity: 2,
          total: 31.86,
        },
        { name: "5/8 Sprocket Hub", cost: 17.31, quantity: 4, total: 69.24 },
        {
          name: "14''*5/8 Keyway Shaft",
          cost: 15.13,
          quantity: 2,
          total: 30.26,
        },
        {
          name: "#25 Roller Chain with Links",
          cost: 15.93,
          quantity: 2,
          total: 31.86,
        },
        {
          name: "Optical Rotary Encoder",
          cost: 16.54,
          quantity: 2,
          total: 33.08,
        },
        {
          name: "6mm Flanged Coupling Connecter x4",
          cost: 5.94,
          quantity: 1,
          total: 5.94,
        },
        {
          name: "#25 Chain Sprocket 10 Tooth",
          cost: 9.87,
          quantity: 1,
          total: 9.87,
        },
        {
          name: "AlveyTech#25 Chain Sprocket 80 Tooth",
          cost: 15.415,
          quantity: 2,
          total: 30.83,
        },
        {
          name: "4inch Turntable Bearing",
          cost: 13.8,
          quantity: 2,
          total: 27.6,
        },
        { name: "#410 Chain", cost: 12.74, quantity: 2, total: 25.48 },
        { name: "16T #410 Sprocket", cost: 13.8, quantity: 2, total: 27.6 },
        {
          name: "18''*12'' Mild Steel Plate",
          cost: 68.7,
          quantity: 1,
          total: 68.7,
        },
      ],
    },
    {
      category: "Free",
      data: [
        {
          name: "2 inch slab of wood (30x30)",
          cost: 50,
          quantity: 1,
          total: 50,
        },
        { name: "Realsense D457", cost: 300, quantity: 1, total: 300 },
        { name: "Raspberry PI", cost: 80, quantity: 1, total: 80 },
        { name: "UWB Modules", cost: 50, quantity: 4, total: 200 },
        {
          name: "Various Wires and Small Electronics",
          cost: 20,
          quantity: 1,
          total: 20,
        },
        {
          name: "Nema 23 Stepper Motor 2.4NM",
          cost: 27.61,
          quantity: 2,
          total: 55.22,
        },
        {
          name: "TB6600 Stepper Motor Driver",
          cost: 10.61,
          quantity: 2,
          total: 21.22,
        },
        {
          name: "6-Inch Trolley Wheel x2",
          cost: 30.8,
          quantity: 2,
          total: 61.6,
        },
        { name: "Various bolts", cost: 20, quantity: 1, total: 20 },
        { name: "T-Slot Nut", cost: 10.99, quantity: 1, total: 10.99 },
        { name: "Extra M6 bolts", cost: 8.5, quantity: 1, total: 8.5 },
        { name: "WAGOs (for sensors)", cost: 10, quantity: 1, total: 10 },
        {
          name: "PN00218-CYT14 Motor Controller",
          cost: 95.01,
          quantity: 1,
          total: 95.01,
        },
        { name: "UWB Module", cost: 225, quantity: 1, total: 225 },
      ],
    },
  ],
  getPurchasedTotal: function () {
    return this.categories
      .filter((cat) => cat.category !== "Free")
      .reduce(
        (sum, cat) =>
          sum + cat.data.reduce((catSum, item) => catSum + item.total, 0),
        0
      );
  },
  getFreeTotal: function () {
    return this.categories
      .filter((cat) => cat.category === "Free")
      .reduce(
        (sum, cat) =>
          sum + cat.data.reduce((catSum, item) => catSum + item.total, 0),
        0
      );
  },
  getGrandTotal: function () {
    return this.getPurchasedTotal() + this.getFreeTotal();
  },
};

