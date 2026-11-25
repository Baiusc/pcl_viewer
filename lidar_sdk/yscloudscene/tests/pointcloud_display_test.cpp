#include "component/pointcloud/pointcloud_display.h"
#include "business/pointcloud_parser.h"
#include "business/pointcloud_parser_imp.h"

#include <cstring>

#include <QApplication>
#include <QOpenGLDebugLogger>
#include <QSurfaceFormat>
#include <QtTest>

using namespace ys_scene;

namespace {
PointcloudRenderFramePtr MakeFrame(int id, bool empty = false, int radius = 10)
{
    auto frame = std::make_shared<PointcloudRenderFrame>();
    auto points = std::make_shared<PointcloudRenderFrame::Points>();
    frame->frame_id = id;
    frame->sequence = NextPointcloudRenderSequence();
    if (!empty) {
        for (int x = -radius; x <= radius; ++x) {
            for (int y = -radius; y <= radius; ++y) {
                you_see::CartesianPoint point{};
                point.x.f = static_cast<float>(x);
                point.y.f = static_cast<float>(y);
                points->push_back(point);
                frame->colors.emplace_back(255, 0, 255);
            }
        }
        frame->bounds_min = QVector3D(-radius, -radius, 0);
        frame->bounds_max = QVector3D(radius, radius, 0);
        frame->bounds_diagonal = (frame->bounds_max - frame->bounds_min).length();
    }
    frame->points = std::move(points);
    return frame;
}

int CountMagentaPixels(const QImage& image)
{
    int count = 0;
    for (int y = 0; y < image.height(); ++y) {
        for (int x = 0; x < image.width(); ++x) {
            const QColor color = image.pixelColor(x, y);
            if (color.red() > 230 && color.blue() > 230 && color.green() < 30) {
                ++count;
            }
        }
    }
    return count;
}
} // namespace

class PointcloudDisplayTest : public QObject
{
    Q_OBJECT

private slots:
    void gammaReceiveToFramebuffer()
    {
        qRegisterMetaType<PointcloudRenderFramePtr>();
        PointcloudDisplay display;
        display.resize(640, 480);
        display.show();
        QVERIFY(QTest::qWaitForWindowExposed(&display));
        QVERIFY(display.isValid());
        display.SetView(PointcloudDisplay::ViewOrientation::Top);
        const int baseline = CountMagentaPixels(display.grabFramebuffer());

        PointcloudParser parser;
        parser.SetOptionalSlotNumber(static_cast<int>(OptionalSlotNumber::KSlot800));
        parser.Clear();
        parser.Insert(ColorScaleElement(0.0, Qt::magenta), false);
        parser.Insert(ColorScaleElement(1.0, Qt::magenta), false);
        parser.Update();
        for (auto* processor : parser.GetProcessors()) {
            processor->SetROI(ROISelectType::kROIReset, {});
            connect(processor, &PointcloudParserImp::RenderFrameReady, processor,
                [mailbox = display.GetRenderMailbox()](PointcloudRenderFramePtr frame) {
                    mailbox->Publish(std::move(frame));
                }, Qt::DirectConnection);
        }

        constexpr int kPixels = 120;
        std::vector<uint8_t> bytes(offsetof(you_see::PointCloudNetPacket, points) +
            kPixels * sizeof(you_see::CartesianPoint));
        auto* packet = reinterpret_cast<you_see::PointCloudNetPacket*>(bytes.data());
        packet->total_slots = 800;
        packet->frame_index = 3;
        packet->current_slot = 799;
        parser.PointcloudReceive2(0, packet, kPixels, nullptr);
        packet->frame_index = 0;
        for (int slot = 0; slot < 800; ++slot) {
            packet->current_slot = slot;
            for (int pixel = 0; pixel < kPixels; ++pixel) {
                you_see::CartesianPoint point{};
                point.x.f = 5.0f + slot * 0.01f;
                point.y.f = -10.0f + pixel * 0.16f;
                point.z.f = 1.0f;
                point.reflectivity = 128;
                std::memcpy(packet->points + pixel * sizeof(point), &point, sizeof(point));
            }
            parser.PointcloudReceive2(0, packet, kPixels, nullptr);
        }
        QTRY_VERIFY(CountMagentaPixels(display.grabFramebuffer()) > baseline + 20);
        parser.Stop();
    }

    void uploadClearAndSwitchMirrors()
    {
        PointcloudDisplay display;
        display.resize(640, 480);
        display.show();
        QVERIFY(QTest::qWaitForWindowExposed(&display));
        QVERIFY2(display.isValid(), "A working desktop OpenGL context is required for this test.");
        display.SetView(PointcloudDisplay::ViewOrientation::Top);

        QStringList errors;
        QOpenGLDebugLogger logger;
        display.makeCurrent();
        QVERIFY(logger.initialize());
        connect(&logger, &QOpenGLDebugLogger::messageLogged, this,
                [&errors](const QOpenGLDebugMessage& message) {
                    if (message.type() == QOpenGLDebugMessage::ErrorType) {
                        errors.push_back(message.message());
                    }
                });
        logger.startLogging(QOpenGLDebugLogger::SynchronousLogging);
        display.doneCurrent();

        const QImage baseline = display.grabFramebuffer();
        QVERIFY(!baseline.isNull());
        const int baseline_pixels = CountMagentaPixels(baseline);

        display.UpdateCloud(MakeFrame(0));
        const QImage rendered = display.grabFramebuffer();
        QVERIFY(CountMagentaPixels(rendered) > baseline_pixels + 20);

        display.UpdateCloud(MakeFrame(0, true));
        const QImage cleared = display.grabFramebuffer();
        QVERIFY(CountMagentaPixels(cleared) <= baseline_pixels + 10);

        // Mode switches clear visibility while retaining each mode's VBO storage.
        for (int cycle = 0; cycle < 3; ++cycle) {
            display.SetShowAllMirrors(true);
            for (int id = 0; id < 4; ++id) {
                display.UpdateCloud(MakeFrame(id));
            }
            QVERIFY(CountMagentaPixels(display.grabFramebuffer()) > baseline_pixels + 20);
            display.SetShowAllMirrors(false);
            display.UpdateCloud(MakeFrame(0));
            QVERIFY(CountMagentaPixels(display.grabFramebuffer()) > baseline_pixels + 20);
        }
        display.makeCurrent();
        logger.stopLogging();
        display.doneCurrent();
        QVERIFY2(errors.isEmpty(), qPrintable(errors.join('\n')));
    }

    void coalesceBeforeUploadAndReuseStorage()
    {
        PointcloudDisplay display;
        display.resize(640, 480);
        // Frames arriving before initializeGL are retained without touching OpenGL.
        display.UpdateCloud(MakeFrame(0));
        display.UpdateCloud(MakeFrame(0));
        QCOMPARE(display.GetRenderStatistics().uploaded_frames, std::size_t(0));
        display.show();
        QVERIFY(QTest::qWaitForWindowExposed(&display));
        QVERIFY(display.isValid());
        display.SetView(PointcloudDisplay::ViewOrientation::Top);
        display.grabFramebuffer();
        QCOMPARE(display.GetRenderStatistics().uploaded_frames, std::size_t(1));
        QCOMPARE(display.GetRenderStatistics().created_buffer_pairs, std::size_t(1));
        QCOMPARE(display.GetRenderStatistics().buffer_growths, std::size_t(1));

        const auto late_old_frame = MakeFrame(0);
        for (int i = 0; i < 50; ++i) {
            display.UpdateCloud(MakeFrame(i % 4));
        }
        // The newest frame is empty; a late older task must not resurrect its points.
        display.UpdateCloud(MakeFrame(0, true));
        display.UpdateCloud(late_old_frame);
        QCOMPARE(display.GetRenderStatistics().uploaded_frames, std::size_t(1));
        QVERIFY(CountMagentaPixels(display.grabFramebuffer()) < 10);
        QCOMPARE(display.GetRenderStatistics().uploaded_frames, std::size_t(2));
        display.UpdateCloud(late_old_frame);
        display.grabFramebuffer();
        QCOMPARE(display.GetRenderStatistics().uploaded_frames, std::size_t(2));

        display.UpdateCloud(MakeFrame(0));
        QVERIFY(CountMagentaPixels(display.grabFramebuffer()) > 20);
        display.UpdateCloud(MakeFrame(0, false, 2));
        display.grabFramebuffer();
        QCOMPARE(display.GetRenderStatistics().created_buffer_pairs, std::size_t(1));
        QCOMPARE(display.GetRenderStatistics().buffer_growths, std::size_t(1));
        display.UpdateCloud(MakeFrame(0, false, 12));
        display.grabFramebuffer();
        QCOMPARE(display.GetRenderStatistics().created_buffer_pairs, std::size_t(1));
        QCOMPARE(display.GetRenderStatistics().buffer_growths, std::size_t(2));

        display.SetShowAllMirrors(true);
        for (int i = 0; i < 40; ++i) {
            display.UpdateCloud(MakeFrame(i % 4));
        }
        const auto before = display.GetRenderStatistics().uploaded_frames;
        display.grabFramebuffer();
        QCOMPARE(display.GetRenderStatistics().uploaded_frames, before + 4);
        QCOMPARE(display.GetRenderStatistics().created_buffer_pairs, std::size_t(5));
        const auto growths = display.GetRenderStatistics().buffer_growths;
        for (int cycle = 0; cycle < 2; ++cycle) {
            display.SetShowAllMirrors(false);
            QVERIFY(CountMagentaPixels(display.grabFramebuffer()) < 10);
            display.UpdateCloud(MakeFrame(0));
            display.grabFramebuffer();
            display.SetShowAllMirrors(true);
            for (int i = 0; i < 4; ++i) {
                display.UpdateCloud(MakeFrame(i));
            }
            display.grabFramebuffer();
        }
        QCOMPARE(display.GetRenderStatistics().created_buffer_pairs, std::size_t(5));
        QCOMPARE(display.GetRenderStatistics().buffer_growths, growths);
    }

    void mailboxOutlivesUninitializedWidgetSafely()
    {
        auto display = std::make_unique<PointcloudDisplay>();
        auto mailbox = display->GetRenderMailbox();
        mailbox->Publish(MakeFrame(0));
        display.reset();
        mailbox->Publish(MakeFrame(0));
        for (const auto& frame : mailbox->Take()) {
            QVERIFY(!frame);
        }
    }
};

int main(int argc, char** argv)
{
    auto format = QSurfaceFormat::defaultFormat();
    format.setOption(QSurfaceFormat::DebugContext);
    QSurfaceFormat::setDefaultFormat(format);
    QApplication app(argc, argv);
    Q_INIT_RESOURCE(ys);
    PointcloudDisplayTest test;
    return QTest::qExec(&test, argc, argv);
}

#include "pointcloud_display_test.moc"
